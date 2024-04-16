import os
import unittest

import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat

from mplib import ArticulatedModel, Pose
from mplib.collision_detection import fcl
from mplib.kinematics.pinocchio import PinocchioModel

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
    "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
    "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
}

ALL_LINKS = [
    "panda_link0",
    "panda_link1",
    "panda_link2",
    "panda_link3",
    "panda_link4",
    "panda_link5",
    "panda_link6",
    "panda_link7",
    "panda_link8",
    "panda_hand",
    "panda_leftfinger",
    "panda_rightfinger",
]

ALL_JOINTS = [
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


class TestArticulation(unittest.TestCase):
    def setUp(self):
        self.robot = ArticulatedModel(
            PANDA_SPEC["urdf"],
            PANDA_SPEC["srdf"],
            link_names=[],
            joint_names=[],
            convex=True,
            verbose=False,
        )

        self.target_pose = [0.4, 0.3, 0.12, 0, 1, 0, 0]
        self.init_qpos = np.array([0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0, 0])
        self.joint_limits = [
            [-2.8973, 2.8973],
            [-1.7628, 1.7628],
            [-2.8973, 2.8973],
            [-3.0718, -0.0698],
            [-2.8973, 2.8973],
            [-0.0175, 3.7525],
            [-2.8973, 2.8973],
            [0, 0.04],
            [0, 0.04],
        ]

    def test_get_base_pose(self):
        base_pose = self.robot.get_base_pose()
        self.assertIsInstance(base_pose, Pose)
        self.assertEqual(base_pose.distance(Pose()), 0)

    def test_get_fcl_model(self):
        fcl_model = self.robot.get_fcl_model()
        self.assertIsInstance(fcl_model, fcl.FCLModel)

    def test_get_move_group_end_effectors(self):
        end_effectors = self.robot.get_move_group_end_effectors()
        self.assertIsInstance(end_effectors, list)
        self.assertEqual(end_effectors, ALL_LINKS)

    def test_get_move_group_joint_indices(self):
        joint_indices = self.robot.get_move_group_joint_indices()
        self.assertIsInstance(joint_indices, list)
        self.assertEqual(joint_indices, list(range(9)))
        self.robot.set_move_group("panda_hand")
        joint_indices = self.robot.get_move_group_joint_indices()
        self.assertEqual(joint_indices, list(range(7)))

    def test_get_move_group_joint_names(self):
        joint_names = self.robot.get_move_group_joint_names()
        self.assertIsInstance(joint_names, list)
        self.assertEqual(joint_names, ALL_JOINTS)
        self.robot.set_move_group("panda_link4")
        joint_names = self.robot.get_move_group_joint_names()
        self.assertEqual(joint_names, ALL_JOINTS[:4])

    def test_get_move_group_qpos_dim(self):
        qpos_dim = self.robot.get_move_group_qpos_dim()
        self.assertIsInstance(qpos_dim, int)
        self.assertEqual(qpos_dim, 9)
        self.robot.set_move_group("panda_hand")
        qpos_dim = self.robot.get_move_group_qpos_dim()
        self.assertEqual(qpos_dim, 7)

    def test_get_pinocchio_model(self):
        pinocchio_model = self.robot.get_pinocchio_model()
        self.assertIsInstance(pinocchio_model, PinocchioModel)

    def test_get_qpos(self):
        qpos = self.robot.get_qpos()
        self.assertIsInstance(qpos, np.ndarray)
        self.assertEqual(len(qpos), len(self.robot.get_user_joint_names()))
        self.robot.set_qpos(np.arange(len(qpos)) / 10.0, full=True)
        qpos = self.robot.get_qpos()
        self.assertTrue(np.allclose(qpos, np.arange(len(qpos)) / 10.0))

    def test_get_user_joint_names(self):
        joint_names = self.robot.get_user_joint_names()
        self.assertIsInstance(joint_names, list)
        self.assertEqual(joint_names, ALL_JOINTS)

    def test_get_user_link_names(self):
        link_names = self.robot.get_user_link_names()
        self.assertIsInstance(link_names, list)
        self.assertEqual(link_names, ALL_LINKS)

    def test_set_base_pose(self):
        pose = np.arange(7) / 10.0
        pose[3:] /= np.linalg.norm(pose[3:])
        pose = Pose(pose[:3], pose[3:])
        self.robot.set_base_pose(pose)
        self.assertAlmostEqual(self.robot.get_base_pose().distance(pose), 0, places=3)

    def test_update_SRDF(self):
        self.robot.update_SRDF("")  # should not raise error
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()
