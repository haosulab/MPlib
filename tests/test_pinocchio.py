import os
import unittest
import numpy as np
from mplib.pymp.kinematics.pinocchio import PinocchioModel

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
  "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
  "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
  "move_group": "panda_hand",
}

class TestPinocchioModel(unittest.TestCase):
  def setUp(self):
    # Create a PinocchioModel instance for testing
    self.model = PinocchioModel(PANDA_SPEC["urdf"], [0, 0, -9.81], verbose=False)

  def test_get_leaf_links(self):
    # Test the getLeafLinks method
    leaf_links = self.model.get_leaf_links()
    self.assertEqual(len(leaf_links), 2)  # two finger links

  def test_set_link_order(self):
    # Test the setLinkOrder method
    names = ['panda_link0',
             'panda_link1',
             'panda_link3',
             'panda_link2',
             'panda_link5',
             'panda_link4',
             'panda_link7',
             'panda_link8',
             'panda_link6',
             'panda_hand']
    self.model.set_link_order(names)
    self.assertEqual(self.model.get_link_names(), names)

  def test_set_joint_order(self):
    # Test the setJointOrder method
    names = ['panda_joint1',
             'panda_joint2',
             'panda_finger_joint1',
             'panda_joint4',
             'panda_joint3',
             'panda_joint5',
             'panda_joint7',
             'panda_joint6',
             'panda_finger_joint2']
    self.model.set_joint_order(names)
    self.assertEqual(self.model.get_joint_names(), names)

  def test_get_joint_dim(self):
    # Test the getJointDim method
    for i in range(len(self.model.get_joint_names())):
      self.assertEqual(self.model.get_joint_dim(i), 1)  # all joints are 1D

  def test_get_joint_parent(self):
    names = ['panda_joint1',
             'panda_joint2',
             'panda_finger_joint1',
             'panda_joint4',
             'panda_joint3',
             'panda_joint5',
             'panda_joint7',
             'panda_joint6',
             'panda_finger_joint2']
    self.model.set_joint_order(names)
    expected = [0, 1, 7, 3, 2, 4, 6, 5, 7]
    for i in range(len(self.model.get_joint_names())):
      self.assertEqual(self.model.get_joint_parent(i), expected[i])

if __name__ == "__main__":
  unittest.main()
