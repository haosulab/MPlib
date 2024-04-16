#!/usr/bin/env python3

import numpy as np
import transforms3d

from mplib.examples.demo_setup import DemoSetup


class ConstrainedPlanningDemo(DemoSetup):
    """
    This demo shows the planner's ability to plan with constraints.
    For this particular demo, we move to several poses while pointing the end effector
    roughly 15 degrees w.r.t. -z axis.
    """

    def __init__(self):
        """Set up the scene and load the robot"""
        super().__init__()
        self.setup_scene()
        self.load_robot()
        self.setup_planner()

    def add_point_cloud(self):
        """Add some random obstacles to make the planning more challenging"""
        import trimesh

        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        all_pts = np.concatenate(
            [points + [-0.65, -0.1, 0.4], points + [0.55, 0, 0.1]], axis=0
        )
        self.planner.update_point_cloud(all_pts, resolution=0.02)
        return

    def get_eef_z(self):
        """Helper function for constraint"""
        ee_idx = self.planner.link_name_2_idx[self.planner.move_group]
        ee_pose = self.planner.robot.get_pinocchio_model().get_link_pose(ee_idx)
        mat = transforms3d.quaternions.quat2mat(ee_pose[3:])
        return mat[:, 2]

    def make_f(self):
        """
        Create a constraint function that takes in a qpos and outputs a scalar.
        A valid constraint function should evaluates to 0 when the constraint
        is satisfied.

        See [ompl constrained planning](https://ompl.kavrakilab.org/constrainedPlanning.html)
        for more details.
        """

        # constraint function ankor
        def f(x, out):
            self.planner.robot.set_qpos(x)
            out[0] = (
                self.get_eef_z().dot(np.array([0, 0, -1])) - 0.966
            )  # maintain 15 degrees w.r.t. -z axis

        # constraint function ankor end
        return f

    def make_j(self):
        """
        Create the jacobian of the constraint function w.r.t. qpos.
        This is needed because the planner uses the jacobian to project a random sample
        to the constraint manifold.
        """

        # constraint jacobian ankor
        def j(x, out):
            full_qpos = self.planner.pad_move_group_qpos(x)
            jac = self.planner.robot.get_pinocchio_model().compute_single_link_jacobian(
                full_qpos, len(self.planner.move_group_joint_indices) - 1
            )
            rot_jac = jac[3:, self.planner.move_group_joint_indices]
            for i in range(len(self.planner.move_group_joint_indices)):
                out[i] = np.cross(rot_jac[:, i], self.get_eef_z()).dot(
                    np.array([0, 0, -1])
                )

        # constraint jacobian ankor end
        return j

    def demo(self):
        """
        We first plan with constraints to three poses, then plan without constraints to
        the same poses.
        While not always the case, sometimes without constraints,
        the end effector will tilt almost upside down.
        """
        # this starting pose has the end effector tilted roughly 15 degrees
        starting_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.88, 0.78, 0, 0]
        self.robot.set_qpos(starting_qpos)
        self.planner.robot.set_qpos(starting_qpos[:7])
        # all these poses are constrain compatible (roughly 15 degrees w.r.t. -z axis)
        poses = [
            [-0.4, -0.3, 0.28, 0.0704682, -0.5356872, 0.8342834, 0.1097478],
            [0.6, 0.1, 0.44, 0.0704682, -0.5356872, -0.8342834, -0.1097478],
            [0, -0.3, 0.5, 0.1304237, -0.9914583, 0, 0],
        ]

        # add some point cloud to make the planning more challenging
        # so we can see the effect of no constraint
        self.add_point_cloud()

        # with constraint
        print(
            "with constraint. all movements roughly maintain 15 degrees w.r.t. -z axis"
        )
        for pose in poses:
            result = self.planner.plan_pose(
                pose,
                self.robot.get_qpos(),
                time_step=1 / 250,
                constraint_function=self.make_f(),
                constraint_jacobian=self.make_j(),
                constraint_tolerance=0.05,
            )
            if result["status"] != "Success":
                print(result["status"])
                return -1
            self.follow_path(result)

        # without constraint
        print(
            "without constraint. certain movements can sometimes tilt the end effector"
            " almost upside down"
        )
        for pose in poses:
            result = self.planner.plan_pose(
                pose,
                self.robot.get_qpos(),
                time_step=1 / 250,
                simplify=False,
            )
            if result["status"] != "Success":
                print(result["status"])
                return -1
            self.follow_path(result)


if __name__ == "__main__":
    demo = ConstrainedPlanningDemo()
    demo.demo()
