import os
import unittest

import numpy as np
from transforms3d.quaternions import mat2quat

from mplib.pymp.collision_detection.fcl import FCLModel
from mplib.pymp.kinematics.pinocchio import PinocchioModel

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
    "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
    "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
    "move_group": "panda_hand",
}


class TestFCLModel(unittest.TestCase):
    def setUp(self):
        # Create a FCLModel instance for testing
        self.model = FCLModel(PANDA_SPEC["urdf"], verbose=False)
        self.pinocchio_model = PinocchioModel(PANDA_SPEC["urdf"], verbose=False)
        self.collision_link_names = [
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

        qpos = np.zeros(len(self.pinocchio_model.get_joint_names()))
        self.pinocchio_model.compute_forward_kinematics(qpos)
        for i, link_name in enumerate(self.collision_link_names):
            link_idx = self.pinocchio_model.get_link_names().index(link_name)
            link_pose = self.pinocchio_model.get_link_pose(link_idx)
            self.model.get_collision_objects()[i].shapes[0].set_transformation(
                link_pose
            )

    def test_get_collision_objects(self):
        self.assertEqual(
            self.model.get_collision_link_names(), self.collision_link_names
        )
        for i, link_name in enumerate(self.collision_link_names):
            pinocchio_idx = self.pinocchio_model.get_link_names().index(link_name)
            pos = self.model.get_collision_objects()[i].shapes[0].get_translation()
            quat = mat2quat(
                self.model.get_collision_objects()[i].shapes[0].get_rotation()
            )
            pinocchio_pose = self.pinocchio_model.get_link_pose(pinocchio_idx)
            self.assertTrue(np.allclose(pos, pinocchio_pose[:3]))
            self.assertAlmostEqual(abs(quat.dot(pinocchio_pose[3:])), 1)

    def test_remove_collision_pairs_from_srdf(self):
        old_collision_pairs = self.model.get_collision_pairs()
        self.model.remove_collision_pairs_from_srdf(PANDA_SPEC["srdf"])
        new_collision_pairs = self.model.get_collision_pairs()
        for pair in new_collision_pairs:
            self.assertIn(pair, old_collision_pairs)

    def test_collision(self):
        collisions = self.model.collide_full()
        self.assertGreater(len(collisions), 0)
        # some of them are collisions
        is_collision = [collision.is_collision() for collision in collisions]
        self.assertTrue(any(is_collision))
        self.model.remove_collision_pairs_from_srdf(PANDA_SPEC["srdf"])
        collisions = self.model.collide_full()
        is_collision = [collision.is_collision() for collision in collisions]
        self.assertFalse(any(is_collision))


if __name__ == "__main__":
    unittest.main()
