import os
import unittest
import numpy as np
from transforms3d.quaternions import qmult, qinverse, quat2axangle
from mplib.pymp.kinematics.pinocchio import PinocchioModel

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

MYCOBOT_SPEC = {
  "urdf": f"{FILE_ABS_DIR}/../data/mycobot/mycobot_with_gripper_parallel.urdf",
  "srdf": f"{FILE_ABS_DIR}/../data/mycobot/mycobot_with_gripper_parallel.srdf",
  "move_group": "link_eef",
  "joint_limits": [[-2.879793, 2.879793],
                   [-2.879793, 2.879793],
                   [-2.879793, 2.879793],
                   [-2.879793, 2.879793],
                   [-2.879793, 2.879793],
                   [-2.879793, 3.054326],
                   [-0.7, 0.15],
                   [-0.7, 0.15],
                   [-0.7, 0.15],
                   [-0.7, 0.15],
                   [-0.7, 0.15],
                   [-0.7, 0.15]]
}

class TestPinocchioModel(unittest.TestCase):
  def setUp(self):
    # Create a PinocchioModel instance for testing
    self.model = PinocchioModel(MYCOBOT_SPEC["urdf"], [0, 0, -9.81], verbose=False)
    self.joint_limits = MYCOBOT_SPEC["joint_limits"]
    self.target_pose = np.array([0.19157131, -0.01226188, -0.05912056, 0.65888356, 0.2610391, 0.61917366, -0.33816419])
    self.target_qpos = np.array([0.554, -1.931, -1.156, 0.543, -1.744, -3.022])
    self.target_qpos = np.concatenate((self.target_qpos, np.zeros(6)))
    self.ee_idx = self.model.get_link_names().index(MYCOBOT_SPEC["move_group"])

  def sample_qpos(self):
    qpos = []
    for limit in self.joint_limits:
      qpos.append(np.random.uniform(limit[0], limit[1]))
    return np.array(qpos)
  
  def test_two_jacobian_methods(self):
    ee_idx = self.model.get_link_names().index(MYCOBOT_SPEC["move_group"])
    for local in [True, False]:
      for _ in range(10):
        qpos = self.sample_qpos()
        self.model.compute_forward_kinematics(qpos)
        self.model.compute_full_jacobian(qpos)
        analytical_jacobian_method1 = self.model.get_link_jacobian(ee_idx, local=local)
        analytical_jacobian_method2 = self.model.compute_single_link_jacobian(qpos, ee_idx, local=local)
        self.assertTrue(np.allclose(analytical_jacobian_method1, analytical_jacobian_method2))

  def get_numerical_jacobian(self, qpos, link_idx, epsilon=1e-3):
    jacobian = np.zeros((6, len(qpos)))
    for i in range(len(qpos)):
      perturbation = np.zeros(len(qpos))
      perturbation[i] = epsilon
      self.model.compute_forward_kinematics(qpos + perturbation)
      pose_plus = self.model.get_link_pose(link_idx)
      self.model.compute_forward_kinematics(qpos - perturbation)
      pose_minus = self.model.get_link_pose(link_idx)
      
      position_diff = pose_plus[:3] - pose_minus[:3]
      jacobian[:3, i] = position_diff / (2 * epsilon)

      orientation_plus = pose_plus[3:]
      orientation_minus = pose_minus[3:]
      # get the difference quaternion
      orientation_diff = qmult(orientation_plus, qinverse(orientation_minus))
      # get the axis-angle representation
      axis, angle = quat2axangle(orientation_diff)
      jacobian[3:, i] = angle / (2 * epsilon) * axis

      self.model.compute_forward_kinematics(qpos)
      curr_pos = self.model.get_link_pose(link_idx)[:3]
      speed_due_to_rotation = np.cross(jacobian[3:, i], curr_pos)
      jacobian[:3, i] -= speed_due_to_rotation

    return jacobian

  def test_link_jacobian(self):
    # idea is to calculate the jacobian numerically by perturbing the joint angles one by one
    # and comparing the result to the analytical jacobian
    for _ in range(10):
      qpos = self.sample_qpos()
      self.model.compute_forward_kinematics(qpos)
      analytical_jacobian = self.model.compute_single_link_jacobian(qpos, self.ee_idx)
      numerical_jacobian = self.get_numerical_jacobian(qpos, self.ee_idx)
      self.assertTrue(np.allclose(analytical_jacobian, numerical_jacobian, atol=1e-3))

  def test_fk(self):
    self.model.compute_forward_kinematics(self.target_qpos)
    self.assertTrue(np.allclose(self.model.get_link_pose(self.ee_idx), self.target_pose, atol=1e-3))

  def test_ik(self):
    result, success, error = self.model.compute_IK_CLIK(self.ee_idx, self.target_pose, np.zeros(12))
    self.model.compute_forward_kinematics(result)
    self.assertTrue(np.allclose(self.model.get_link_pose(self.ee_idx), self.target_pose, atol=1e-3))

if __name__ == "__main__":
  unittest.main()
