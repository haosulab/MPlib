from __future__ import annotations

import os
from collections.abc import Callable
from typing import Optional, Sequence

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
from transforms3d.quaternions import mat2quat, quat2mat

from mplib.pymp import ArticulatedModel, PlanningWorld
from mplib.pymp.collision_detection import WorldCollisionResult
from mplib.pymp.planning import ompl


class Planner:
    """Motion planner"""

    # TODO(jigu): default joint vel and acc limits
    # TODO(jigu): how does user link names and joint names are exactly used?
    # constructor ankor
    def __init__(
        self,
        urdf: str,
        move_group: str,
        srdf: str = "",
        package_keyword_replacement: str = "",
        user_link_names: Sequence[str] = [],
        user_joint_names: Sequence[str] = [],
        joint_vel_limits: Optional[Sequence[float] | np.ndarray] = None,
        joint_acc_limits: Optional[Sequence[float] | np.ndarray] = None,
        **kwargs,
    ):
        # constructor ankor end
        """Motion planner for robots.

        Args:
            urdf: Unified Robot Description Format file.
            user_link_names: names of links, the order matters.
                If empty, all links will be used.
            user_joint_names: names of the joints to plan.
                If empty, all active joints will be used.
            move_group: target link to move, usually the end-effector.
            joint_vel_limits: maximum joint velocities for time parameterization,
                which should have the same length as
            joint_acc_limits: maximum joint accelerations for time parameterization,
                which should have the same length as
            srdf: Semantic Robot Description Format file.
        References:
            http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html

        """
        if joint_vel_limits is None:
            joint_vel_limits = []
        if joint_acc_limits is None:
            joint_acc_limits = []
        self.urdf = urdf
        self.srdf = srdf
        if self.srdf == "" and os.path.exists(urdf.replace(".urdf", ".srdf")):
            self.srdf = urdf.replace(".urdf", ".srdf")
            print(f"No SRDF file provided but found {self.srdf}")

        # replace package:// keyword if exists
        urdf = self.replace_package_keyword(package_keyword_replacement)

        self.robot = ArticulatedModel(
            self.urdf,
            self.srdf,
            link_names=user_link_names,
            joint_names=user_joint_names,
            convex=kwargs.get("convex", False),
            verbose=False,
        )
        self.pinocchio_model = self.robot.get_pinocchio_model()
        self.user_link_names = self.pinocchio_model.get_link_names()
        self.user_joint_names = self.pinocchio_model.get_joint_names()

        self.planning_world = PlanningWorld(
            [self.robot],
            ["robot"],
            kwargs.get("normal_objects", []),
            kwargs.get("normal_object_names", []),
        )
        self.acm = self.planning_world.get_allowed_collision_matrix()

        self.joint_name_2_idx = {}
        for i, joint in enumerate(self.user_joint_names):
            self.joint_name_2_idx[joint] = i
        self.link_name_2_idx = {}
        for i, link in enumerate(self.user_link_names):
            self.link_name_2_idx[link] = i

        if self.srdf == "":
            self.generate_collision_pair()
            self.robot.update_SRDF(self.srdf)

        assert (
            move_group in self.user_link_names
        ), f"end-effector not found as one of the links in {self.user_link_names}"
        self.move_group = move_group
        self.robot.set_move_group(self.move_group)
        self.move_group_joint_indices = self.robot.get_move_group_joint_indices()

        self.joint_types = self.pinocchio_model.get_joint_types()
        self.joint_limits = np.concatenate(self.pinocchio_model.get_joint_limits())
        self.joint_vel_limits = (
            joint_vel_limits
            if len(joint_vel_limits)
            else np.ones(len(self.move_group_joint_indices))
        )
        self.joint_acc_limits = (
            joint_acc_limits
            if len(joint_acc_limits)
            else np.ones(len(self.move_group_joint_indices))
        )
        self.move_group_link_id = self.link_name_2_idx[self.move_group]
        assert len(self.joint_vel_limits) == len(self.joint_acc_limits), (
            f"length of joint_vel_limits ({len(self.joint_vel_limits)}) =/= "
            f"length of joint_acc_limits ({len(self.joint_acc_limits)})"
        )
        assert len(self.joint_vel_limits) == len(self.move_group_joint_indices), (
            f"length of joint_vel_limits ({len(self.joint_vel_limits)}) =/= "
            f"length of move_group ({len(self.move_group_joint_indices)})"
        )
        assert len(self.joint_vel_limits) <= len(self.joint_limits), (
            f"length of joint_vel_limits ({len(self.joint_vel_limits)}) > "
            f"number of total joints ({len(self.joint_limits)})"
        )

        # Mask for joints that have equivalent values (revolute joints with range > 2pi)
        self.equiv_joint_mask = [
            t.startswith("JointModelR") for t in self.joint_types
        ] & (self.joint_limits[:, 1] - self.joint_limits[:, 0] > 2 * np.pi)

        self.planner = ompl.OMPLPlanner(world=self.planning_world)

    def replace_package_keyword(self, package_keyword_replacement):
        """
        some ROS URDF files use package:// keyword to refer the package dir
        replace it with the given string (default is empty)

        Args:
            package_keyword_replacement: the string to replace 'package://' keyword
        """
        rtn_urdf = self.urdf
        with open(self.urdf) as in_f:
            content = in_f.read()
            if "package://" in content:
                rtn_urdf = self.urdf.replace(".urdf", "_package_keyword_replaced.urdf")
                content = content.replace("package://", package_keyword_replacement)
                if not os.path.exists(rtn_urdf):
                    with open(rtn_urdf, "w") as out_f:
                        out_f.write(content)
        return rtn_urdf

    def generate_collision_pair(self, sample_time=1000000, echo_freq=10):
        """
        We read the srdf file to get the link pairs that should not collide.
        If not provided, we need to randomly sample configurations
        to find the link pairs that will always collide.
        """
        print(
            "Since no SRDF file is provided. We will first detect link pairs that will"
            " always collide. This may take several minutes."
        )

        import xml.etree.ElementTree as ET
        from xml.dom import minidom

        root = ET.Element("robot")
        robot_name = self.urdf.split("/")[-1].split(".")[0]
        root.set("name", robot_name)
        self.srdf = self.urdf.replace(".urdf", ".srdf")

        for link1, link2 in self.pinocchio_model.get_adjacent_links():
            print(f"Ignore collision pair: ({link1}, {link2}), " "reason: adjacent")
            collision = ET.SubElement(root, "disable_collisions")
            collision.set("link1", link1)
            collision.set("link2", link2)
            collision.set("reason", "adjacent")

        leaf_links = self.pinocchio_model.get_leaf_links()
        for i in range(len(leaf_links) - 1):
            j = i + 1
            link1 = leaf_links[i]
            link2 = leaf_links[j]
            print(
                f"Ignore collision pair: ({link1}, {link2}), "
                "reason: leaf links, so highly likely to be gripper parts"
            )
            collision = ET.SubElement(root, "disable_collisions")
            collision.set("link1", link1)
            collision.set("link2", link2)
            collision.set("reason", "leaf")

        with open(self.srdf, "w") as srdf_file:
            srdf_file.write(
                minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
            )
            srdf_file.close()
        print("Saving the SRDF file to %s" % self.srdf)

    def distance_6D(self, p1, q1, p2, q2):
        """
        compute the distance between two poses

        Args:
            p1: position of pose 1
            q1: quaternion of pose 1
            p2: position of pose 2
            q2: quaternion of pose 2
        """
        return np.linalg.norm(p1 - p2) + min(
            np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2)
        )

    def wrap_joint_limit(self, qpos: np.ndarray) -> bool:
        """
        Checks if the joint configuration can be wrapped to be within the joint limits.
        For revolute joints, the joint angle is wrapped to be within [q_min, q_min+2*pi)

        :param qpos: joint positions, angles of revolute joints might be modified.
            If not within_limits (returns False), qpos might not be fully wrapped.
        :return: whether qpos can be wrapped to be within the joint limits.
        """
        for i, (q, joint_type, qlimit) in enumerate(
            zip(qpos, self.joint_types, self.joint_limits)
        ):
            if joint_type.startswith("JointModelR"):  # revolute joint
                if -1e-3 <= q - qlimit[0] < 0:
                    continue
                q -= 2 * np.pi * np.floor((q - qlimit[0]) / (2 * np.pi))
                qpos[i] = q  # clip qpos
                if q > qlimit[1] + 1e-3:
                    return False
            else:
                if q < qlimit[0] - 1e-3 or q > qlimit[1] + 1e-3:
                    return False
        return True

    def pad_move_group_qpos(self, qpos, articulation=None):
        """
        If qpos contains only the move_group joints, return qpos padded with
        current values of the remaining joints of articulation.
        Otherwise, verify number of joints and return.

        :param qpos: joint positions
        :param articulation: the articulation to get qpos from. If None, use self.robot
        :return: joint positions with full dof
        """
        if articulation is None:
            articulation = self.robot

        if (ndim := len(qpos)) == articulation.get_move_group_qpos_dim():
            tmp = articulation.get_qpos().copy()
            tmp[:ndim] = qpos
            qpos = tmp

        assert len(qpos) == len(self.joint_limits), (
            f"length of qpos ({len(qpos)}) != "
            f"number of total joints ({len(self.joint_limits)})"
        )
        return qpos

    def check_for_collision(
        self,
        collision_function,
        state: Optional[np.ndarray] = None,
    ) -> list[WorldCollisionResult]:
        """
        Helper function to check for collision

        :param state: all planned articulations qpos state. If None, use current qpos.
        :return: A list of collisions.
        """
        planned_articulations = self.planning_world.get_planned_articulations()
        if len(planned_articulations) > 1:
            raise NotImplementedError("Only support 1 planned articulation now")
        articulation = planned_articulations[0]

        # first save the current qpos
        old_qpos = articulation.get_qpos()
        # Ensure state contains full dof
        state = self.pad_move_group_qpos(
            old_qpos if state is None else state, articulation
        )
        # set robot to new qpos
        articulation.set_qpos(state, True)
        # check for collision
        collisions = collision_function()
        # reset qpos
        articulation.set_qpos(old_qpos, True)
        return collisions

    def check_for_self_collision(
        self,
        state: Optional[np.ndarray] = None,
    ) -> list[WorldCollisionResult]:
        """
        Check if the robot is in self-collision.

        :param state: all planned articulations qpos state. If None, use current qpos.
        :return: A list of collisions.
        """
        return self.check_for_collision(self.planning_world.self_collide, state)

    def check_for_env_collision(
        self,
        state: Optional[np.ndarray] = None,
    ) -> list[WorldCollisionResult]:
        """
        Check if the robot is in collision with the environment

        :param state: all planned articulations qpos state. If None, use current qpos.
        :return: A list of collisions.
        """
        return self.check_for_collision(self.planning_world.collide_with_others, state)

    def IK(
        self,
        goal_pose: np.ndarray,
        start_qpos: np.ndarray,
        mask: Optional[list[bool] | np.ndarray] = None,
        *,
        n_init_qpos: int = 20,
        threshold: float = 1e-3,
        return_closest: bool = False,
        verbose: bool = False,
    ) -> tuple[str, list[np.ndarray] | np.ndarray | None]:
        """
        Compute inverse kinematics

        :param goal_pose: goal pose (xyz, wxyz), (7,) np.floating np.ndarray.
        :param start_qpos: initial configuration, (ndof,) np.floating np.ndarray.
        :param mask: qpos mask to disable IK sampling, (ndof,) bool np.ndarray.
        :param n_init_qpos: number of random initial configurations to sample.
        :param threshold: distance_6D threshold for marking sampled IK as success.
                          distance_6D is position error norm + quaternion error norm.
        :param return_closest: whether to return the qpos that is closest to start_qpos,
                               considering equivalent joint values.
        :param verbose: whether to print collision info if any collision exists.
        :return: (status, q_goals)
            status: IK status, "Success" if succeeded.
            q_goals: list of sampled IK qpos, (ndof,) np.floating np.ndarray.
                IK is successful if q_goals is not None.
                If return_closest, q_goals is np.ndarray if successful
                and None if not successful.
        """
        if mask is None:
            mask = []

        move_link_idx = self.link_name_2_idx[self.move_group]
        move_joint_idx = self.move_group_joint_indices
        self.robot.set_qpos(start_qpos, True)

        min_dis = 1e9
        q_goals = []
        qpos = start_qpos
        for _ in range(n_init_qpos):
            ik_qpos, ik_success, ik_error = self.pinocchio_model.compute_IK_CLIK(
                move_link_idx, goal_pose, qpos, mask
            )
            success = ik_success and self.wrap_joint_limit(ik_qpos)

            if success:
                # check collision
                self.planning_world.set_qpos_all(ik_qpos[move_joint_idx])
                if len(collisions := self.planning_world.collide_full()) > 0:
                    success = False
                    if verbose:
                        for collision in collisions:
                            print(
                                f"Collision between {collision.link_name1} of entity "
                                f"{collision.object_name1} with {collision.link_name2} "
                                f"of entity {collision.object_name2}"
                            )

            if success:
                self.pinocchio_model.compute_forward_kinematics(ik_qpos)
                new_pose = self.pinocchio_model.get_link_pose(move_link_idx)
                tmp_dis = self.distance_6D(
                    goal_pose[:3], goal_pose[3:], new_pose[:3], new_pose[3:]
                )
                if tmp_dis < min_dis:
                    min_dis = tmp_dis
                if tmp_dis < threshold:
                    for q_goal in q_goals:
                        if (
                            np.linalg.norm(
                                q_goal[move_joint_idx] - ik_qpos[move_joint_idx]
                            )
                            < 0.1
                        ):
                            break  # not unique ik_qpos
                    else:
                        q_goals.append(ik_qpos)

            qpos = self.pinocchio_model.get_random_configuration()
            qpos[mask] = start_qpos[mask]  # use start_qpos for disabled joints

        if len(q_goals) != 0:
            status = "Success"
        elif min_dis != 1e9:
            status = f"IK Failed! Distance {min_dis} is greater than {threshold=}."
            return status, None
        else:
            status = "IK Failed! Cannot find valid solution."
            return status, None

        if return_closest:
            q_goals = np.asarray(q_goals)  # [N, ndof]
            start_qpos = np.asarray(start_qpos)[None]  # [1, ndof]

            # Consider equivalent joint values
            q1 = q_goals[:, self.equiv_joint_mask]  # [N, n_equiv_joint]
            q2 = q1 + 2 * np.pi  # equivalent joints
            start_q = start_qpos[:, self.equiv_joint_mask]  # [1, n_equiv_joint]

            # Mask where q2 is valid and closer to start_q
            q2_closer_mask = (
                q2 < self.joint_limits[:, 1][None, self.equiv_joint_mask]
            ) & (np.abs(q1 - start_q) > np.abs(q2 - start_q))  # [N, n_equiv_joint]
            # Convert q_goals to equivalent joint values closest to start_qpos
            q_goals[:, self.equiv_joint_mask] = np.where(q2_closer_mask, q2, q1)

            q_goals = q_goals[np.linalg.norm(q_goals - start_qpos, axis=1).argmin()]
        return status, q_goals

    def TOPP(self, path, step=0.1, verbose=False):
        """
        Time-Optimal Path Parameterization

        Args:
            path: numpy array of shape (n, dof)
            step: step size for the discretization
            verbose: if True, will print the log of TOPPRA
        """

        N_samples = path.shape[0]
        dof = path.shape[1]
        assert dof == len(self.joint_vel_limits)
        assert dof == len(self.joint_acc_limits)
        ss = np.linspace(0, 1, N_samples)
        path = ta.SplineInterpolator(ss, path)
        pc_vel = constraint.JointVelocityConstraint(self.joint_vel_limits)
        pc_acc = constraint.JointAccelerationConstraint(self.joint_acc_limits)
        instance = algo.TOPPRA(
            [pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel"
        )
        jnt_traj = instance.compute_trajectory()
        if jnt_traj is None:
            raise RuntimeError("Fail to parameterize path")
        ts_sample = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration / step))
        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        return ts_sample, qs_sample, qds_sample, qdds_sample, jnt_traj.duration

    # TODO: change method name to align with PlanningWorld API?
    def update_point_cloud(self, points, resolution=1e-3, name="scene_pcd"):
        """
        Adds a point cloud as a collision object with given name to world.
        If the ``name`` is the same, the point cloud is simply updated.

        :param points: points, numpy array of shape (n, 3)
        :param resolution: resolution of the point OcTree
        :param name: name of the point cloud collision object
        """
        self.planning_world.add_point_cloud(name, points, resolution)

    def remove_point_cloud(self, name="scene_pcd") -> bool:
        """
        Removes the point cloud collision object with given name

        :param name: name of the point cloud collision object
        :return: ``True`` if success, ``False`` if normal object with given name doesn't
            exist
        """
        return self.planning_world.remove_normal_object(name)

    def update_attached_object(
        self,
        collision_geometry,
        pose,
        name="attached_geom",
        art_name=None,
        link_id=-1,
    ):
        """
        Attach given object (w/ collision geometry) to specified link of articulation

        :param collision_geometry: FCL collision geometry
        :param pose: attaching pose (relative pose from attached link to object),
                     [x,y,z,qw,qx,qy,qz]
        :param name: name of the attached geometry.
        :param art_name: name of the articulated object to attach to.
                         If None, attach to self.robot.
        :param link_id: if not provided, the end effector will be the target.
        """
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_object(
            name,
            collision_geometry,
            self.robot.get_name() if art_name is None else art_name,
            link_id,
            pose,
        )

    def update_attached_sphere(self, radius, pose, art_name=None, link_id=-1):
        """
        Attach a sphere to some link

        :param radius: radius of the sphere
        :param pose: attaching pose (relative pose from attached link to object),
                     [x,y,z,qw,qx,qy,qz]
        :param art_name: name of the articulated object to attach to.
                         If None, attach to self.robot.
        :param link_id: if not provided, the end effector will be the target.
        """
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_sphere(
            radius,
            self.robot.get_name() if art_name is None else art_name,
            link_id,
            pose,
        )

    def update_attached_box(self, size, pose, art_name=None, link_id=-1):
        """
        Attach a box to some link

        :param size: box side length
        :param pose: attaching pose (relative pose from attached link to object),
                     [x,y,z,qw,qx,qy,qz]
        :param art_name: name of the articulated object to attach to.
                         If None, attach to self.robot.
        :param link_id: if not provided, the end effector will be the target.
        """
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_box(
            size, self.robot.get_name() if art_name is None else art_name, link_id, pose
        )

    def update_attached_mesh(self, mesh_path, pose, art_name=None, link_id=-1):
        """
        Attach a mesh to some link

        :param mesh_path: path to a mesh file
        :param pose: attaching pose (relative pose from attached link to object),
                     [x,y,z,qw,qx,qy,qz]
        :param art_name: name of the articulated object to attach to.
                         If None, attach to self.robot.
        :param link_id: if not provided, the end effector will be the target.
        """
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_mesh(
            mesh_path,
            self.robot.get_name() if art_name is None else art_name,
            link_id,
            pose,
        )

    def detach_object(self, name="attached_geom", also_remove=False) -> bool:
        """
        Detact the attached object with given name

        :param name: object name to detach
        :param also_remove: whether to also remove object from world
        :return: ``True`` if success, ``False`` if the object with given name is not
            attached
        """
        return self.planning_world.detach_object(name, also_remove)

    def set_base_pose(self, pose):
        """
        tell the planner where the base of the robot is w.r.t the world frame

        Args:
            pose: [x,y,z,qw,qx,qy,qz] pose of the base
        """
        self.robot.set_base_pose(pose)

    def remove_normal_object(self, name) -> bool:
        """returns true if the object was removed, false if it was not found"""
        return self.planning_world.remove_normal_object(name)

    def plan_qpos_to_qpos(
        self,
        goal_qposes: list[np.ndarray],
        current_qpos: np.ndarray,
        *,
        time_step: float = 0.1,
        rrt_range: float = 0.1,
        planning_time: float = 1,
        fix_joint_limits: bool = True,
        fixed_joint_indices: Optional[list[int]] = None,
        simplify: bool = True,
        constraint_function: Optional[Callable] = None,
        constraint_jacobian: Optional[Callable] = None,
        constraint_tolerance: float = 1e-3,
        verbose: bool = False,
    ) -> dict[str, str | np.ndarray | np.float64]:
        """
        Plan a path from a specified joint position to a goal pose

        Args:
            goal_qposes: list of target joint configurations, [(ndof,)]
            current_qpos: current joint configuration (either full or move_group joints)
            mask: mask for IK. When set, the IK will leave certain joints out of
                planning
            time_step: time step for TOPP
            rrt_range: step size for RRT
            planning_time: time limit for RRT
            fix_joint_limits: if True, will clip the joint configuration to be within
                the joint limits
            simplify: whether the planned path will be simplified.
                (constained planning does not support simplification)
            constraint_function: evals to 0 when constraint is satisfied
            constraint_jacobian: jacobian of constraint_function
            constraint_tolerance: tolerance for constraint_function
            fixed_joint_indices: list of indices of joints that are fixed during
                planning
            verbose: if True, will print the log of OMPL and TOPPRA
        """
        if fixed_joint_indices is None:
            fixed_joint_indices = []

        if fix_joint_limits:
            current_qpos = np.clip(
                current_qpos, self.joint_limits[:, 0], self.joint_limits[:, 1]
            )
        current_qpos = self.pad_move_group_qpos(current_qpos)

        self.robot.set_qpos(current_qpos, True)
        collisions = self.planning_world.collide_full()
        if len(collisions) > 0:
            print("Invalid start state!")
            for collision in collisions:
                print(f"{collision.link_name1} and {collision.link_name2} collide!")

        move_joint_idx = self.move_group_joint_indices

        goal_qpos_ = [goal_qposes[i][move_joint_idx] for i in range(len(goal_qposes))]

        fixed_joints = set()
        for joint_idx in fixed_joint_indices:
            fixed_joints.add(ompl.FixedJoint(0, joint_idx, current_qpos[joint_idx]))

        assert len(current_qpos[move_joint_idx]) == len(goal_qpos_[0])
        status, path = self.planner.plan(
            current_qpos[move_joint_idx],
            goal_qpos_,
            time=planning_time,
            range=rrt_range,
            fixed_joints=fixed_joints,
            simplify=simplify,
            constraint_function=constraint_function,
            constraint_jacobian=constraint_jacobian,
            constraint_tolerance=constraint_tolerance,
            verbose=verbose,
        )

        if status == "Exact solution":
            if verbose:
                ta.setup_logging("INFO")
            else:
                ta.setup_logging("WARNING")
            times, pos, vel, acc, duration = self.TOPP(path, time_step)
            return {
                "status": "Success",
                "time": times,
                "position": pos,
                "velocity": vel,
                "acceleration": acc,
                "duration": duration,
            }
        else:
            return {"status": f"RRTConnect Failed. {status}"}

    def transform_goal_to_wrt_base(self, goal_pose):
        base_pose = self.robot.get_base_pose()
        base_tf = np.eye(4)
        base_tf[0:3, 3] = base_pose[:3]
        base_tf[0:3, 0:3] = quat2mat(base_pose[3:])
        goal_tf = np.eye(4)
        goal_tf[0:3, 3] = goal_pose[:3]
        goal_tf[0:3, 0:3] = quat2mat(goal_pose[3:])
        goal_tf = np.linalg.inv(base_tf).dot(goal_tf)
        new_goal_pose = np.zeros(7)
        new_goal_pose[:3] = goal_tf[0:3, 3]
        new_goal_pose[3:] = mat2quat(goal_tf[0:3, 0:3])
        return new_goal_pose

    def plan_qpos_to_pose(
        self,
        goal_pose: np.ndarray,
        current_qpos: np.ndarray,
        mask: Optional[list[bool] | np.ndarray] = None,
        *,
        time_step: float = 0.1,
        rrt_range: float = 0.1,
        planning_time: float = 1,
        fix_joint_limits: bool = True,
        wrt_world: bool = True,
        simplify: bool = True,
        constraint_function: Optional[Callable] = None,
        constraint_jacobian: Optional[Callable] = None,
        constraint_tolerance: float = 1e-3,
        verbose: bool = False,
    ) -> dict[str, str | np.ndarray | np.float64]:
        """
        plan from a start configuration to a goal pose of the end-effector

        Args:
            goal_pose: [x,y,z,qw,qx,qy,qz] pose of the goal
            current_qpos: current joint configuration (either full or move_group joints)
            mask: if the value at a given index is True, the joint is *not* used in the
                IK
            time_step: time step for TOPPRA (time parameterization of path)
            rrt_range: step size for RRT
            planning_time: time limit for RRT
            fix_joint_limits: if True, will clip the joint configuration to be within
                the joint limits
            wrt_world: if true, interpret the target pose with respect to
                the world frame instead of the base frame
            verbose: if True, will print the log of OMPL and TOPPRA
        """
        if mask is None:
            mask = []

        if fix_joint_limits:
            current_qpos = np.clip(
                current_qpos, self.joint_limits[:, 0], self.joint_limits[:, 1]
            )
        current_qpos = self.pad_move_group_qpos(current_qpos)

        if wrt_world:
            goal_pose = self.transform_goal_to_wrt_base(goal_pose)

        # we need to take only the move_group joints when planning
        # idx = self.move_group_joint_indices

        ik_status, goal_qpos = self.IK(goal_pose, current_qpos, mask)
        if ik_status != "Success":
            return {"status": ik_status}

        if verbose:
            print("IK results:")
            for i in range(len(goal_qpos)):
                print(goal_qpos[i])

        # goal_qpos_ = [goal_qpos[i][move_joint_idx] for i in range(len(goal_qpos))]
        self.robot.set_qpos(current_qpos, True)

        ik_status, goal_qpos = self.IK(goal_pose, current_qpos, mask)
        if ik_status != "Success":
            return {"status": ik_status}

        if verbose:
            print("IK results:")
            for i in range(len(goal_qpos)):
                print(goal_qpos[i])

        return self.plan_qpos_to_qpos(
            goal_qpos,
            current_qpos,
            time_step=time_step,
            rrt_range=rrt_range,
            planning_time=planning_time,
            fix_joint_limits=fix_joint_limits,
            simplify=simplify,
            constraint_function=constraint_function,
            constraint_jacobian=constraint_jacobian,
            constraint_tolerance=constraint_tolerance,
            verbose=verbose,
        )

    # plan_screw ankor
    def plan_screw(
        self,
        goal_pose: np.ndarray,
        current_qpos: np.ndarray,
        *,
        qpos_step: float = 0.1,
        time_step: float = 0.1,
        wrt_world: bool = True,
        verbose: bool = False,
    ) -> dict[str, str | np.ndarray | np.float64]:
        # plan_screw ankor end
        """
        Plan from a start configuration to a goal pose of the end-effector using
        screw motion

        Args:
            goal_pose: [x, y, z, qw, qx, qy, qz] pose of the goal
            current_qpos: current joint configuration (either full or move_group joints)
            qpos_step: size of the random step for RRT
            time_step: time step for the discretization
            wrt_world: if True, interpret the target pose with respect to the
                world frame instead of the base frame
            verbose: if True, will print the log of TOPPRA
        """
        current_qpos = self.pad_move_group_qpos(current_qpos.copy())
        self.robot.set_qpos(current_qpos, True)

        if wrt_world:
            goal_pose = self.transform_goal_to_wrt_base(goal_pose)

        def pose7D2mat(pose):
            mat = np.eye(4)
            mat[0:3, 3] = pose[:3]
            mat[0:3, 0:3] = quat2mat(pose[3:])
            return mat

        def skew(vec):
            return np.array([
                [0, -vec[2], vec[1]],
                [vec[2], 0, -vec[0]],
                [-vec[1], vec[0], 0],
            ])

        def pose2exp_coordinate(pose: np.ndarray) -> tuple[np.ndarray, float]:
            def rot2so3(rotation: np.ndarray):
                assert rotation.shape == (3, 3)
                if np.isclose(rotation.trace(), 3):
                    return np.zeros(3), 1
                if np.isclose(rotation.trace(), -1):
                    return np.zeros(3), -1e6
                theta = np.arccos((rotation.trace() - 1) / 2)
                omega = (
                    1
                    / 2
                    / np.sin(theta)
                    * np.array([
                        rotation[2, 1] - rotation[1, 2],
                        rotation[0, 2] - rotation[2, 0],
                        rotation[1, 0] - rotation[0, 1],
                    ]).T
                )
                return omega, theta

            omega, theta = rot2so3(pose[:3, :3])
            if theta < -1e5:
                return omega, theta
            ss = skew(omega)
            inv_left_jacobian = (
                np.eye(3) / theta
                - 0.5 * ss
                + (1.0 / theta - 0.5 / np.tan(theta / 2)) * ss @ ss
            )
            v = inv_left_jacobian @ pose[:3, 3]
            return np.concatenate([v, omega]), theta

        self.pinocchio_model.compute_forward_kinematics(current_qpos)
        ee_index = self.link_name_2_idx[self.move_group]
        current_p = pose7D2mat(self.pinocchio_model.get_link_pose(ee_index))
        target_p = pose7D2mat(goal_pose)
        relative_transform = target_p @ np.linalg.inv(current_p)

        omega, theta = pose2exp_coordinate(relative_transform)

        if theta < -1e4:
            return {"status": "screw plan failed."}
        omega = omega.reshape((-1, 1)) * theta

        move_joint_idx = self.move_group_joint_indices
        path = [np.copy(current_qpos[move_joint_idx])]

        while True:
            self.pinocchio_model.compute_full_jacobian(current_qpos)
            J = self.pinocchio_model.get_link_jacobian(ee_index, local=False)
            delta_q = np.linalg.pinv(J) @ omega
            delta_q *= qpos_step / (np.linalg.norm(delta_q))
            delta_twist = J @ delta_q

            flag = False
            if np.linalg.norm(delta_twist) > np.linalg.norm(omega):
                ratio = np.linalg.norm(omega) / np.linalg.norm(delta_twist)
                delta_q = delta_q * ratio
                delta_twist = delta_twist * ratio
                flag = True

            current_qpos += delta_q.reshape(-1)
            omega -= delta_twist

            def check_joint_limit(q):
                n = len(q)
                for i in range(n):
                    if (
                        q[i] < self.joint_limits[i][0] - 1e-3
                        or q[i] > self.joint_limits[i][1] + 1e-3
                    ):
                        return False
                return True

            within_joint_limit = check_joint_limit(current_qpos)
            self.planning_world.set_qpos_all(current_qpos[move_joint_idx])
            collide = self.planning_world.collide()

            if np.linalg.norm(delta_twist) < 1e-4 or collide or not within_joint_limit:
                return {"status": "screw plan failed"}

            path.append(np.copy(current_qpos[move_joint_idx]))

            if flag:
                if verbose:
                    ta.setup_logging("INFO")
                else:
                    ta.setup_logging("WARNING")
                times, pos, vel, acc, duration = self.TOPP(np.vstack(path), time_step)
                return {
                    "status": "Success",
                    "time": times,
                    "position": pos,
                    "velocity": vel,
                    "acceleration": acc,
                    "duration": duration,
                }
