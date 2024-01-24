import numpy as np
import unittest
from mplib import Planner

PANDA_SPEC = {
  "urdf": "data/panda/panda.urdf",
  "srdf": "data/panda/panda.srdf",
  "move_group": "panda_hand",
}

class TestPlannerSimple(unittest.TestCase):
  def __init__(self, methodName: str = "runTest") -> None:
    super().__init__(methodName)
    self.planner = Planner(**PANDA_SPEC)
    self.target_pose = [0.4, 0.3, 0.12, 0, 1, 0, 0]
    self.init_qpos = np.array([0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0, 0])

  def get_end_effector_pose(self):
    ee_idx = self.planner.link_name_2_idx[self.planner.move_group]
    return self.planner.robot.get_pinocchio_model().get_link_pose(ee_idx)

  def test_planner(self):
    result_sampling = self.planner.plan_qpos_to_pose(self.target_pose, self.init_qpos)
    self.assertEqual(result_sampling["status"], "Success")
    last_qpos = result_sampling["position"][-1]
    self.planner.robot.set_qpos(last_qpos)
    self.assertTrue(np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-2))

  def test_planner_screw(self):
    result_sampling = self.planner.plan_screw(self.target_pose, self.init_qpos)
    self.assertEqual(result_sampling["status"], "Success")
    last_qpos = result_sampling["position"][-1]
    self.planner.robot.set_qpos(last_qpos)
    self.assertTrue(np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-2))


if __name__ == "__main__":
  unittest.main()
