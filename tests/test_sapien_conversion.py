import os
import unittest

import numpy as np
import sapien.core as sapien
from transforms3d.quaternions import mat2quat

from mplib import Pose
from mplib.sapien_utils.conversion import (
    SapienPlanner,
    SapienPlanningWorld,
    convert_object_name,
)

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
    "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
    "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
    "move_group": "panda_hand",
}


class TestSapienConversion(unittest.TestCase):
    """

    def setUp(self):
        self.engine = sapien.Engine()
        scene_config = sapien.SceneConfig()
        self.scene = self.engine.create_scene(scene_config)
        self.scene.set_timestep(1 / 240)
        self.scene.add_ground(0)
        self.scene.default_physical_material = self.scene.create_physical_material(
            1, 1, 0
        )

        # robot
        loader: sapien.URDFLoader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot: sapien.Articulation = loader.load(PANDA_SPEC["urdf"])
        self.robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))
        self.robot.set_qpos([0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0])

        # red box is the target we want to grab
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], material=[1, 0, 0])
        self.red_cube = builder.build(name="red_cube")
        self.red_cube.set_pose(sapien.Pose([0.7, 0, 0.06]))

        # green box is the landing pad on which we want to place the red box
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.04, 0.04, 0.005])
        builder.add_box_visual(half_size=[0.04, 0.04, 0.005], material=[0, 1, 0])
        self.green_cube = builder.build(name="green_cube")
        self.green_cube.set_pose(sapien.Pose([0.4, 0.3, 0.005]))

        # blue box is the obstacle we want to avoid
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.2, 0.1])
        builder.add_box_visual(half_size=[0.05, 0.2, 0.1], material=[0, 0, 1])
        self.blue_cube = builder.build(name="blue_cube")
        self.blue_cube.set_pose(sapien.Pose([0.55, 0, 0.1]))

        self.planning_world = SapienPlanningWorld(self.scene, [self.robot])
        self.planner = SapienPlanner(self.planning_world, "panda_hand")
        # disable collision between the base and the ground
        self.planning_world.get_allowed_collision_matrix().set_entry(
            "panda_link0", "ground_1", True
        )

        self.target_pose = Pose([0.7, 0, 0.3], [0, 1, 0, 0])

    def follow_path(self, result):
        # for now just set the qpos directly
        full_qpos = self.robot.get_qpos()
        full_qpos[: len(self.planner.move_group_joint_indices)] = result["position"][-1]
        self.robot.set_qpos(full_qpos)

    def test_articulation_planned(self):
        self.assertTrue(self.planning_world.is_articulation_planned(self.robot))
        self.planning_world.set_articulation_planned(self.robot, False)
        self.assertFalse(self.planning_world.is_articulation_planned(self.robot))

        # after toggling back to True, we should be able to plan to a pose
        self.planning_world.set_articulation_planned(self.robot, True)
        result = self.planner.plan_pose(self.target_pose, self.robot.get_qpos())
        self.assertTrue(result["status"] == "Success")

        self.follow_path(result)
        # check if the robot is at the target pose
        self.planner.update_from_simulation()
        ee_idx = self.planner.user_link_names.index("panda_hand")
        ee_pose = self.planner.robot.get_pinocchio_model().get_link_pose(ee_idx)
        self.assertAlmostEqual(self.target_pose.distance(ee_pose), 0, places=3)

    def test_add_remove_obj(self):
        self.assertTrue(self.planning_world.has_object(self.red_cube))
        self.planning_world.remove_object(self.red_cube)
        self.assertFalse(self.planning_world.has_object(self.red_cube))

        # after removing the blue cube, the plan should succeed
        self.planning_world.remove_object(self.blue_cube)
        result = self.planner.plan_screw(self.target_pose, self.robot.get_qpos())
        self.assertTrue(result["status"] == "Success")
        # but after adding it back, the plan should fail
        self.planning_world.add_object(self.blue_cube)
        result = self.planner.plan_screw(self.target_pose, self.robot.get_qpos())
        self.assertFalse(result["status"] == "Success")

    def test_attach_detach_obj(self):
        # the red cube should be at around [0.7, 0, 0.06] before picking up
        self.assertTrue(np.allclose(self.red_cube.get_pose().p, [0.7, 0, 0.06]))

        result = self.planner.plan_pose(self.target_pose, self.robot.get_qpos())
        self.assertTrue(result["status"] == "Success")
        self.follow_path(result)

        self.assertFalse(self.planning_world.is_object_attached(self.red_cube))
        ee_idx = self.planner.user_link_names.index("panda_hand")
        self.planning_world.attach_object(self.red_cube, self.robot, ee_idx)
        self.assertTrue(self.planning_world.is_object_attached(self.red_cube))

        lift_pose = self.target_pose
        lift_pose.p[2] += 0.2
        result = self.planner.plan_pose(lift_pose, self.robot.get_qpos())
        self.follow_path(result)

        # the red cube should be at around [0.7, 0, 0.26] after picking up
        red_cube_name = convert_object_name(self.red_cube)
        red_cube_attach = self.planning_world.get_object(red_cube_name)
        expected_pose = Pose(p=[0.7, 0, 0.26])
        self.assertTrue(expected_pose.distance(red_cube_attach.pose) < 0.1)

    def test_allowed_collision(self):
        # initially should fail
        self.planning_world.add_object(self.blue_cube)
        result = self.planner.plan_screw(self.target_pose, self.robot.get_qpos())
        self.assertFalse(result["status"] == "Success")

        # after allowing collision, should succeed
        # get the link entity that is robot's end effector

        sapien_links = self.robot.get_links()
        sapien_link_names = [link.get_name() for link in sapien_links]
        sapien_eef = sapien_links[sapien_link_names.index("panda_hand")]
        self.planning_world.set_allowed_collision(self.blue_cube, sapien_eef, True)
        self.planning_world.remove_object(self.blue_cube)
        result = self.planner.plan_screw(self.target_pose, self.robot.get_qpos())
        self.assertTrue(result["status"] == "Success")

    """


if __name__ == "__main__":
    unittest.main()
