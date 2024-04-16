import os
import unittest

import numpy as np
from transforms3d.quaternions import qinverse, qmult, quat2axangle

from mplib.kinematics.pinocchio import PinocchioModel

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
    "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
    "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
    "move_group": "panda_hand",
}


class TestPinocchioModel(unittest.TestCase):
    def setUp(self):
        # Create a PinocchioModel instance for testing
        self.model = PinocchioModel(PANDA_SPEC["urdf"], verbose=False)
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

    def test_get_leaf_links(self):
        # Test the getLeafLinks method
        leaf_links = self.model.get_leaf_links()
        self.assertEqual(len(leaf_links), 2)  # two finger links

    def test_set_link_order(self):
        # Test the setLinkOrder method
        names = [
            "panda_link0",
            "panda_link1",
            "panda_link3",
            "panda_link2",
            "panda_link5",
            "panda_link4",
            "panda_link7",
            "panda_link8",
            "panda_link6",
            "panda_hand",
        ]
        self.model.set_link_order(names)
        self.assertEqual(self.model.get_link_names(), names)

    def test_set_joint_order(self):
        # Test the setJointOrder method
        names = [
            "panda_joint1",
            "panda_joint2",
            "panda_finger_joint1",
            "panda_joint4",
            "panda_joint3",
            "panda_joint5",
            "panda_joint7",
            "panda_joint6",
            "panda_finger_joint2",
        ]
        self.model.set_joint_order(names)
        self.assertEqual(self.model.get_joint_names(), names)

    def test_get_joint_dim(self):
        # Test the getJointDim method
        for i in range(len(self.model.get_joint_names())):
            self.assertEqual(self.model.get_joint_dim(i), 1)  # all joints are 1D

    def test_get_joint_parent(self):
        names = [
            "panda_joint1",
            "panda_joint2",
            "panda_finger_joint1",
            "panda_joint4",
            "panda_joint3",
            "panda_joint5",
            "panda_joint7",
            "panda_joint6",
            "panda_finger_joint2",
        ]
        self.model.set_joint_order(names)
        expected = [0, 1, 7, 3, 2, 4, 6, 5, 7]
        for i in range(len(self.model.get_joint_names())):
            self.assertEqual(self.model.get_joint_parent(i), expected[i])

    def sample_qpos(self):
        qpos = []
        for limit in self.joint_limits:
            qpos.append(np.random.uniform(limit[0], limit[1]))
        return np.array(qpos)

    def test_two_jacobian_methods(self):
        ee_idx = self.model.get_link_names().index("panda_hand")
        for local in [True, False]:
            for _ in range(10):
                qpos = self.sample_qpos()
                self.model.compute_forward_kinematics(qpos)
                self.model.compute_full_jacobian(qpos)
                analytical_jacobian_method1 = self.model.get_link_jacobian(
                    ee_idx, local=local
                )
                analytical_jacobian_method2 = self.model.compute_single_link_jacobian(
                    qpos, ee_idx, local=local
                )
                self.assertTrue(
                    np.allclose(
                        analytical_jacobian_method1, analytical_jacobian_method2
                    )
                )

    def get_numerical_jacobian(self, qpos, link_idx, epsilon=1e-3):
        jacobian = np.zeros((6, len(qpos)))
        for i in range(len(qpos)):
            perturbation = np.zeros(len(qpos))
            perturbation[i] = epsilon
            self.model.compute_forward_kinematics(qpos + perturbation)
            pose_plus = self.model.get_link_pose(link_idx)
            self.model.compute_forward_kinematics(qpos - perturbation)
            pose_minus = self.model.get_link_pose(link_idx)

            position_diff = pose_plus.get_p() - pose_minus.get_p()
            jacobian[:3, i] = position_diff / (2 * epsilon)

            orientation_plus = pose_plus.get_q()
            orientation_minus = pose_minus.get_q()
            # get the difference quaternion
            orientation_diff = qmult(orientation_plus, qinverse(orientation_minus))
            # get the axis-angle representation
            axis, angle = quat2axangle(orientation_diff)
            jacobian[3:, i] = angle / (2 * epsilon) * axis

            self.model.compute_forward_kinematics(qpos)
            curr_pos = self.model.get_link_pose(link_idx).get_p()
            speed_due_to_rotation = np.cross(jacobian[3:, i], curr_pos)
            jacobian[:3, i] -= speed_due_to_rotation

        return jacobian

    def test_link_jacobian(self):
        # idea is to calculate the jacobian numerically by perturbing the joint angles one by one
        # and comparing the result to the analytical jacobian
        ee_idx = self.model.get_link_names().index("panda_hand")
        for _ in range(10):
            qpos = self.sample_qpos()
            self.model.compute_forward_kinematics(qpos)
            analytical_jacobian = self.model.compute_single_link_jacobian(qpos, ee_idx)
            numerical_jacobian = self.get_numerical_jacobian(qpos, ee_idx)
            self.assertTrue(
                np.allclose(analytical_jacobian, numerical_jacobian, atol=1e-3)
            )


if __name__ == "__main__":
    unittest.main()
