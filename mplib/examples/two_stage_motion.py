import sys

import numpy as np
import sapien.core as sapien

from mplib import Pose
from mplib.collision_detection import fcl
from mplib.examples.demo_setup import DemoSetup


class PlanningDemo(DemoSetup):
    """
    This demo is the same as collision_avoidance.py except we added a track
    for the robot to move along.

    We reach the target in two stages:
    1. First, we move the base while fixing the arm joints
    2. Then, we move the arm while fixing the base joints
    This corresponds to a mobile robot which can move in the x and y direction
    with a manipulator on top
    """

    def __init__(self):
        """
        We have modified the urdf file to include a track for the robot to move along
        Otherwise, the setup is the same as collision_avoidance.py
        """
        super().__init__()
        self.setup_scene()
        self.load_robot(urdf_path="./data/panda/panda_on_rail.urdf")
        link_names = [
            "track_x",
            "track_y",
            "panda_link0",
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
            "panda_link7",
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger",
        ]
        joint_names = [
            "move_x",
            "move_y",
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2",
        ]
        self.setup_planner(
            urdf_path="./data/panda/panda_on_rail.urdf",
            srdf_path="./data/panda/panda_on_rail.srdf",
            link_names=link_names,
            joint_names=joint_names,
            joint_vel_limits=np.ones(9),
            joint_acc_limits=np.ones(9),
        )

        # Set initial joint positions
        init_qpos = [0, 0, 0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0]
        self.robot.set_qpos(init_qpos)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        table = builder.build_kinematic(name="table")
        table.set_pose(sapien.Pose([0.56, 0, -0.025]))

        # boxes
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06])
        red_cube = builder.build(name="red_cube")
        red_cube.set_pose(sapien.Pose([0.7, 0, 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.04, 0.04, 0.005])
        builder.add_box_visual(half_size=[0.04, 0.04, 0.005])
        green_cube = builder.build(name="green_cube")
        green_cube.set_pose(sapien.Pose([0.4, 0.3, 0.005]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.2, 0.1])
        builder.add_box_visual(half_size=[0.05, 0.2, 0.1])
        blue_cube = builder.build(name="blue_cube")
        blue_cube.set_pose(sapien.Pose([0.55, 0, 0.1]))

    def add_point_cloud(self):
        """see collision_avoidance.py for details"""
        import trimesh

        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        points += [0.55, 0, 0.1]
        self.planner.update_point_cloud(points)
        return

    def plan_without_base(self, pose, has_attach=False):
        """a subroutine to plan a path without moving the base"""
        # now do a partial ik to the pose with the base fixed
        status, goal_qposes = self.planner.IK(
            pose, self.robot.get_qpos(), mask=[1, 1, 0, 0, 0, 0, 0, 0, 0]
        )
        if status != "Success":
            print("IK failed")
            sys.exit(1)
        # now fix base and plan a path to the goal
        result = self.planner.plan_qpos(
            goal_qposes,
            self.robot.get_qpos(),
            time_step=1 / 250,
            fixed_joint_indices=range(2),
        )
        return result

    # move_in_two_stage ankor
    def move_in_two_stage(self, pose, has_attach=False):
        """
        first, we do a full IK but only generate motions for the base
        then, do another partial IK for the arm and generate motions for the arm
        """
        # do a full ik to the pose
        status, goal_qposes = self.planner.IK(pose, self.robot.get_qpos())
        if status != "Success":
            print("IK failed")
            sys.exit(1)
        # now fix arm joints and plan a path to the goal
        result = self.planner.plan_qpos(
            goal_qposes,
            self.robot.get_qpos(),
            time_step=1 / 250,
            fixed_joint_indices=range(2, 9),
        )
        # execute the planned path
        self.follow_path(result)
        result = self.plan_without_base(pose, has_attach)
        # execute the planned path
        self.follow_path(result)

    # move_in_two_stage ankor end
    def demo(self):
        """
        We reach the pick up and drop off poses in two stages,
        first by moving the base only and then the arm only
        """
        # pickup ankor
        pickup_pose = [0.7, 0, 0.12, 0, 1, 0, 0]
        delivery_pose = [0.4, 0.3, 0.13, 0, 1, 0, 0]

        self.add_point_cloud()
        # also add the target as a collision object so we don't hit it
        fcl_red_cube = fcl.Box([0.04, 0.04, 0.14])
        collision_object = fcl.CollisionObject(fcl_red_cube, Pose(p=[0.7, 0, 0.07]))
        self.planner.planning_world.add_object("target", collision_object)

        # go above the target
        pickup_pose[2] += 0.2
        self.move_in_two_stage(pickup_pose)
        # pickup ankor end
        self.open_gripper()
        # move down to pick
        self.planner.planning_world.remove_object(
            "target"
        )  # remove the object so we don't report collision
        pickup_pose[2] -= 0.12
        result = self.plan_without_base(pickup_pose)
        self.follow_path(result)
        self.close_gripper()

        # Set planner robot qpos to allow auto-detect touch_links
        self.planner.robot.set_qpos(self.robot.get_qpos(), True)

        # add the attached box to the planning world
        self.planner.update_attached_box([0.04, 0.04, 0.12], Pose(p=[0, 0, 0.14]))

        pickup_pose[2] += 0.12
        result = self.plan_without_base(pickup_pose, has_attach=True)
        self.follow_path(result)
        delivery_pose[2] += 0.2
        # now go to the drop off in two stages
        self.move_in_two_stage(delivery_pose, has_attach=True)
        delivery_pose[2] -= 0.12
        result = self.plan_without_base(delivery_pose, has_attach=True)
        self.follow_path(result)
        self.open_gripper()
        delivery_pose[2] += 0.12
        result = self.plan_without_base(delivery_pose)
        self.follow_path(result)


if __name__ == "__main__":
    demo = PlanningDemo()
    demo.demo()
