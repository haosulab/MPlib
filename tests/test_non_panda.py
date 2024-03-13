import os
import unittest

import numpy as np
import trimesh
from transforms3d.quaternions import mat2quat, quat2mat

import mplib
from mplib import Planner
from mplib.pymp.collision_detection import fcl

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

MYCOBOT_SPEC = {
  "urdf": f"{FILE_ABS_DIR}/../data/mycobot/mycobot_with_gripper_parallel.urdf",
  "srdf": f"{FILE_ABS_DIR}/../data/mycobot/mycobot_with_gripper_parallel.srdf",
  "move_group": "link_eef",
}

class TestMyCobot(unittest.TestCase):
    def setUp(self):
        self.planner = Planner(**MYCOBOT_SPEC)
        self.joint_limits = [[-2.879793, 2.879793],
                            [-2.879793, 2.879793],
                            [-2.879793, 2.879793],
                            [-2.879793, 2.879793],
                            [-2.879793, 2.879793],
                            [-2.879793, 3.054326]]
        self.init_qpos = np.zeros(12)
        self.target_pose = np.array([0.19, -0.01, -0.06, 0.66, 0.26, 0.62, -0.34])

    def get_end_effector_pose(self):
        ee_idx = self.planner.link_name_2_idx[self.planner.move_group]
        return self.planner.robot.get_pinocchio_model().get_link_pose(ee_idx)

    def sample_qpos(self):
        qpos = []
        for limit in self.joint_limits:
            qpos.append(np.random.uniform(limit[0], limit[1]))
        return np.array(qpos)

    def sample_pose(self):
        pos = np.random.uniform(-1, 1, size=3)
        quat = np.random.uniform(0, 1, size=4)
        quat /= np.linalg.norm(quat)
        pose = np.concatenate([pos, quat])
        return pose

    def test_planning_to_pose(self):
        result_sampling = self.planner.plan_qpos_to_pose(
            self.target_pose, self.init_qpos
        )
        self.assertEqual(result_sampling["status"], "Success")
        last_qpos = result_sampling["position"][-1]
        self.planner.robot.set_qpos(last_qpos)
        self.assertTrue(
            np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-3)
        )

    # def test_planning_to_qpos(self):
    #     num_success = 0
    #     expected_least_num_success = 8
    #     for _ in range(20):
    #         target_qpos = self.sample_qpos()[:6]
    #         result_sampling = self.planner.plan_qpos_to_qpos(
    #             [target_qpos], self.init_qpos
    #         )
    #         # might not always be a valid pose, so check if the planner succeeded
    #         if result_sampling["status"] == "Success":
    #             num_success += 1
    #             last_qpos = result_sampling["position"][-1]
    #             self.planner.robot.set_qpos(last_qpos)
    #             self.assertTrue(np.allclose(last_qpos, target_qpos, atol=1e-3))
    #     self.assertGreaterEqual(num_success, expected_least_num_success)

    # def test_planning_screw(self):
    #     result_sampling = self.planner.plan_screw(self.target_pose, self.init_qpos)
    #     self.assertEqual(result_sampling["status"], "Success")
    #     last_qpos = result_sampling["position"][-1]
    #     self.planner.robot.set_qpos(last_qpos)
    #     self.assertTrue(
    #         np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-2)
    #     )

    def test_self_collision(self):
        self_collision_qpos = [0, 1.36, 0, -3, -3, 3]
        self.assertTrue(
            self.planner.check_for_self_collision(
                self.planner.robot, self_collision_qpos
            )
        )
        self_collision_qpos[0] += (
            1  # rotating the robot around the base should not cause self-collision to disappear
        )
        self.assertTrue(
            self.planner.check_for_self_collision(
                self.planner.robot, self_collision_qpos
            )
        )
        collision_free_qpos = [0.554, -1.931, -1.156, 0.543, -1.744, -3.022]
        self.assertFalse(
            self.planner.check_for_self_collision(
                self.planner.robot, collision_free_qpos
            )
        )

    def test_env_collision(self):
        floor = fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
        # create a collision object for the floor, with a 10cm offset in the z direction
        floor_fcl_collision_object = fcl.CollisionObject(
            floor, [0, 0, -0.1], [1, 0, 0, 0]
        )
        # update the planning world with the floor collision object
        self.planner.set_normal_object("floor", floor_fcl_collision_object)

        # this qpos causes several joints to dip below the floor
        env_collision_qpos = [0.554, -1.931, -1.156, 0.543, -1.744, -3.022]
        self.assertTrue(
            self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos)
        )

        # remove the floor and check for env-collision returns no collision
        self.planner.remove_normal_object("floor")
        self.assertFalse(
            self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos)
        )

    # def test_IK(self):
    #     num_success = 0
    #     expected_least_num_success = 10
    #     for _ in range(100):
    #         # generate some random poses that should be within the robot's workspace
    #         pose = self.sample_pose()
    #         status, results = self.planner.IK(pose, self.init_qpos)
    #         num_success += status == "Success"
    #         if status == "Success":
    #             for result_qpos in results:
    #                 self.planner.robot.set_qpos(result_qpos, full=True)
    #                 self.assertTrue(
    #                     np.allclose(self.get_end_effector_pose(), pose, atol=1e-3)
    #                 )
    #     self.assertGreaterEqual(num_success, expected_least_num_success)

    #     # now put down a floor and check that the robot can't reach the pose
    #     floor = fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
    #     # create a collision object for the floor, with a 10cm offset in the z direction
    #     floor_fcl_collision_object = fcl.CollisionObject(
    #         floor, [0, 0, -0.1], [1, 0, 0, 0]
    #     )
    #     status, _ = self.planner.IK([0.4, 0.3, -0.1, 0, 1, 0, 0], self.init_qpos)
    #     self.assertEqual(status, "Success")
    #     self.planner.set_normal_object("floor", floor_fcl_collision_object)
    #     status, _ = self.planner.IK([0.4, 0.3, -0.1, 0, 1, 0, 0], self.init_qpos)
    #     self.assertNotEqual(status, "Success")


if __name__ == "__main__":
    unittest.main()
