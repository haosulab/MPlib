from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Literal, Optional, Sequence

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint

from .collision_detection import WorldCollisionResult
from .collision_detection.fcl import CollisionGeometry, FCLObject
from .planning.ompl import FixedJoint, OMPLPlanner
from .pymp import ArticulatedModel, PlanningWorld, Pose
from .urdf_utils import generate_srdf, replace_urdf_package_keyword


class Planner:
    """Motion planner"""

    # TODO(jigu): default joint vel and acc limits
    # TODO(jigu): how does user link names and joint names are exactly used?
    # constructor ankor
    def __init__(
        self,
        urdf: str | Path,
        move_group: str,
        *,
        srdf: Optional[str | Path] = None,
        new_package_keyword: str = "",
        use_convex: bool = False,
        user_link_names: Sequence[str] = [],
        user_joint_names: Sequence[str] = [],
        joint_vel_limits: Optional[Sequence[float] | np.ndarray] = None,
        joint_acc_limits: Optional[Sequence[float] | np.ndarray] = None,
        objects: list[FCLObject] = [],  # noqa: B006
        verbose: bool = False,
    ):
        # constructor ankor end
        """
        Motion planner for robots.

        References:
            http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html

        :param urdf: Unified Robot Description Format file.
        :param move_group: target link to move, usually the end-effector.
        :param srdf: Semantic Robot Description Format file.
        :param new_package_keyword: the string to replace ``package://`` keyword
        :param use_convex: if True, load collision mesh as convex mesh.
            If mesh is not convex, a ``RuntimeError`` will be raised.
        :param user_link_names: names of links, the order matters.
            If empty, all links will be used.
        :param user_joint_names: names of the joints to plan.
            If empty, all active joints will be used.
        :param joint_vel_limits: maximum joint velocities for time parameterization,
            which should have the same length as ``self.move_group_joint_indices``
        :param joint_acc_limits: maximum joint accelerations for time parameterization,
            which should have the same length as ``self.move_group_joint_indices``
        :param objects: list of FCLObject as non-articulated collision objects
        :param verbose: if True, print verbose logs for debugging
        """
        self.urdf = Path(urdf)
        if srdf is None:
            if (srdf := self.urdf.with_suffix(".srdf")).is_file() or (
                srdf := self.urdf.with_name(self.urdf.stem + "_mplib.srdf")
            ).is_file():
                print(f"No SRDF file provided but found {srdf}")
            else:
                srdf = generate_srdf(urdf, new_package_keyword, verbose=True)
        self.srdf = srdf

        # replace package:// keyword if exists
        self.urdf = replace_urdf_package_keyword(self.urdf, new_package_keyword)

        self.robot = ArticulatedModel(
            str(self.urdf),
            str(self.srdf),
            link_names=user_link_names,  # type: ignore
            joint_names=user_joint_names,  # type: ignore
            convex=use_convex,
            verbose=verbose,
        )
        self.pinocchio_model = self.robot.get_pinocchio_model()
        self.user_link_names = self.pinocchio_model.get_link_names()
        self.user_joint_names = self.pinocchio_model.get_joint_names()

        self.joint_name_2_idx = {}
        for i, joint in enumerate(self.user_joint_names):
            self.joint_name_2_idx[joint] = i
        self.link_name_2_idx = {}
        for i, link in enumerate(self.user_link_names):
            self.link_name_2_idx[link] = i

        assert (
            move_group in self.user_link_names
        ), f"end-effector not found as one of the links in {self.user_link_names}"
        self.move_group = move_group
        self.robot.set_move_group(self.move_group)
        self.move_group_joint_indices = self.robot.get_move_group_joint_indices()

        self.joint_types = self.pinocchio_model.get_joint_types()
        self.joint_limits = np.concatenate(self.pinocchio_model.get_joint_limits())
        if joint_vel_limits is None:
            joint_vel_limits = np.ones(len(self.move_group_joint_indices))
        if joint_acc_limits is None:
            joint_acc_limits = np.ones(len(self.move_group_joint_indices))
        self.joint_vel_limits = joint_vel_limits
        self.joint_acc_limits = joint_acc_limits
        self.move_group_link_id = self.link_name_2_idx[self.move_group]

        assert (
            len(self.joint_vel_limits)
            == len(self.joint_acc_limits)
            == len(self.move_group_joint_indices)
            <= len(self.joint_limits)
        ), (
            "length of joint_vel_limits, joint_acc_limits, and move_group_joint_indices"
            " should equal and be <= number of total joints. "
            f"{len(self.joint_vel_limits)} == {len(self.joint_acc_limits)} "
            f"== {len(self.move_group_joint_indices)} <= {len(self.joint_limits)}"
        )

        # Mask for joints that have equivalent values (revolute joints with range > 2pi)
        self.equiv_joint_mask = [
            t.startswith("JointModelR") for t in self.joint_types
        ] & (self.joint_limits[:, 1] - self.joint_limits[:, 0] > 2 * np.pi)

        self.planning_world = PlanningWorld([self.robot], objects)
        self.acm = self.planning_world.get_allowed_collision_matrix()

        self.planner = OMPLPlanner(world=self.planning_world)

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
        return self.check_for_collision(self.planning_world.check_self_collision, state)

    def check_for_env_collision(
        self,
        state: Optional[np.ndarray] = None,
    ) -> list[WorldCollisionResult]:
        """
        Check if the robot is in collision with the environment

        :param state: all planned articulations qpos state. If None, use current qpos.
        :return: A list of collisions.
        """
        return self.check_for_collision(
            self.planning_world.check_robot_collision, state
        )

    def IK(
        self,
        goal_pose: Pose,
        start_qpos: np.ndarray,
        mask: Optional[Sequence[bool] | np.ndarray] = None,
        *,
        n_init_qpos: int = 20,
        threshold: float = 1e-3,
        return_closest: bool = False,
        verbose: bool = False,
    ) -> tuple[str, list[np.ndarray] | np.ndarray | None]:
        """
        Compute inverse kinematics

        :param goal_pose: goal pose
        :param start_qpos: initial configuration, (ndof,) np.floating np.ndarray.
        :param mask: qpos mask to disable IK sampling, (ndof,) bool np.ndarray.
        :param n_init_qpos: number of random initial configurations to sample.
        :param threshold: distance threshold for marking sampled IK as success.
            distance is position error norm + quaternion error norm.
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

        min_dist = 1e9
        q_goals = []
        qpos = start_qpos
        for _ in range(n_init_qpos):
            ik_qpos, ik_success, _ = self.pinocchio_model.compute_IK_CLIK(
                move_link_idx,
                goal_pose,
                qpos,
                mask,  # type: ignore
            )
            success = ik_success and self.wrap_joint_limit(ik_qpos)

            if success:
                # check collision
                self.planning_world.set_qpos_all(ik_qpos[move_joint_idx])
                if len(collisions := self.planning_world.check_collision()) > 0:
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
                if (dist := goal_pose.distance(new_pose)) < min_dist:
                    min_dist = dist
                if dist < threshold:
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
        elif min_dist != 1e9:
            status = f"IK Failed! Distance {min_dist} is greater than {threshold=}."
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
        ts_sample = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration / step))  # type: ignore
        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        return ts_sample, qs_sample, qds_sample, qdds_sample, jnt_traj.duration  # type: ignore

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
        :return: ``True`` if success, ``False`` if the non-articulation object
            with given name does not exist
        """
        return self.planning_world.remove_object(name)

    def update_attached_object(
        self,
        collision_geometry: CollisionGeometry,
        pose: Pose,
        name="attached_geom",
        art_name=None,
        link_id=-1,
    ):
        """
        Attach given object (w/ collision geometry) to specified link of articulation

        :param collision_geometry: FCL collision geometry
        :param pose: attaching pose (relative pose from attached link to object)
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

    def update_attached_sphere(
        self, radius: float, pose: Pose, art_name=None, link_id=-1
    ):
        """
        Attach a sphere to some link

        :param radius: radius of the sphere
        :param pose: attaching pose (relative pose from attached link to object)
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

    def update_attached_box(
        self,
        size: Sequence[float]
        | np.ndarray[tuple[Literal[3], Literal[1]], np.dtype[np.floating]],
        pose: Pose,
        art_name=None,
        link_id=-1,
    ):
        """
        Attach a box to some link

        :param size: box side length
        :param pose: attaching pose (relative pose from attached link to object)
        :param art_name: name of the articulated object to attach to.
                         If None, attach to self.robot.
        :param link_id: if not provided, the end effector will be the target.
        """
        if link_id == -1:
            link_id = self.move_group_link_id
        self.planning_world.attach_box(
            size,  # type: ignore
            self.robot.get_name() if art_name is None else art_name,
            link_id,
            pose,
        )

    def update_attached_mesh(
        self, mesh_path: str, pose: Pose, art_name=None, link_id=-1
    ):
        """
        Attach a mesh to some link

        :param mesh_path: path to a mesh file
        :param pose: attaching pose (relative pose from attached link to object)
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

    def set_base_pose(self, pose: Pose):
        """
        tell the planner where the base of the robot is w.r.t the world frame

        Args:
            pose: pose of the base
        """
        self.robot.set_base_pose(pose)

    def remove_object(self, name) -> bool:
        """returns true if the object was removed, false if it was not found"""
        return self.planning_world.remove_object(name)

    def plan_qpos(
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
        constraint_function: Optional[Callable[[np.ndarray, np.ndarray], None]] = None,
        constraint_jacobian: Optional[Callable[[np.ndarray, np.ndarray], None]] = None,
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
        collisions = self.planning_world.check_collision()
        if len(collisions) > 0:
            print("Invalid start state!")
            for collision in collisions:
                print(f"{collision.link_name1} and {collision.link_name2} collide!")

        move_joint_idx = self.move_group_joint_indices

        goal_qpos_ = [goal_qposes[i][move_joint_idx] for i in range(len(goal_qposes))]

        fixed_joints = set()
        for joint_idx in fixed_joint_indices:
            fixed_joints.add(FixedJoint(0, joint_idx, current_qpos[joint_idx]))

        assert len(current_qpos[move_joint_idx]) == len(goal_qpos_[0])
        status, path = self.planner.plan(
            current_qpos[move_joint_idx],
            goal_qpos_,
            time=planning_time,
            range=rrt_range,
            fixed_joints=fixed_joints,
            simplify=simplify,
            constraint_function=constraint_function,  # type: ignore
            constraint_jacobian=constraint_jacobian,  # type: ignore
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

    def _transform_goal_to_wrt_base(self, goal_pose: Pose) -> Pose:
        """Converts goal pose from T_world_goal to T_base_goal"""
        return self.robot.get_base_pose().inv() * goal_pose

    def plan_pose(
        self,
        goal_pose: Pose,
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
            goal_pose: pose of the goal
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
            goal_pose = self._transform_goal_to_wrt_base(goal_pose)

        # we need to take only the move_group joints when planning
        # idx = self.move_group_joint_indices

        ik_status, goal_qpos = self.IK(goal_pose, current_qpos, mask)
        if ik_status != "Success":
            return {"status": ik_status}

        if verbose:
            print("IK results:")
            for i in range(len(goal_qpos)):  # type: ignore
                print(goal_qpos[i])  # type: ignore

        return self.plan_qpos(
            goal_qpos,  # type: ignore
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
        goal_pose: Pose,
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
            goal_pose: pose of the goal
            current_qpos: current joint configuration (either full or move_group joints)
            qpos_step: size of the random step
            time_step: time step for the discretization
            wrt_world: if True, interpret the target pose with respect to the
                world frame instead of the base frame
            verbose: if True, will print the log of TOPPRA
        """
        current_qpos = self.pad_move_group_qpos(current_qpos.copy())
        self.robot.set_qpos(current_qpos, True)

        if wrt_world:
            goal_pose = self._transform_goal_to_wrt_base(goal_pose)

        def skew(vec):
            return np.array([
                [0, -vec[2], vec[1]],
                [vec[2], 0, -vec[0]],
                [-vec[1], vec[0], 0],
            ])

        def pose2exp_coordinate(pose: Pose) -> tuple[np.ndarray, float]:
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

            pose_mat = pose.to_transformation_matrix()
            omega, theta = rot2so3(pose_mat[:3, :3])
            if theta < -1e5:
                return omega, theta
            ss = skew(omega)
            inv_left_jacobian = (
                np.eye(3) / theta
                - 0.5 * ss
                + (1.0 / theta - 0.5 / np.tan(theta / 2)) * ss @ ss
            )
            v = inv_left_jacobian @ pose_mat[:3, 3]
            return np.concatenate([v, omega]), theta

        self.pinocchio_model.compute_forward_kinematics(current_qpos)
        ee_index = self.link_name_2_idx[self.move_group]
        # relative_pose = T_base_goal * T_base_link.inv()
        relative_pose = goal_pose * self.pinocchio_model.get_link_pose(ee_index).inv()
        omega, theta = pose2exp_coordinate(relative_pose)

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
            collide = self.planning_world.is_state_colliding()

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
