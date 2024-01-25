import numpy as np
import unittest
from mplib import Planner
import mplib

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

  def test_check_joint_limit(self, tolerance=1e-2):
    # Panda has the following limits:
    limits = [[-2.8973, 2.8973],
              [-1.7628, 1.7628],
              [-2.8973, 2.8973],
              [-3.0718, -0.0698],
              [-2.8973, 2.8973],
              [-0.0175, 3.7525],
              [-2.8973, 2.8973]]
    for _ in range(100):
      qpos = np.random.uniform(-np.pi, np.pi, size=7)
      in_limit = True
      for joint_angle, limit in zip(qpos, limits):
        if joint_angle < limit[0]:
          joint_angle += 2 * np.pi - tolerance
        elif joint_angle > limit[1]:
          joint_angle -= 2 * np.pi + tolerance
        if not (limit[0] <= joint_angle <= limit[1]):
          in_limit = False
          break

      self.assertEqual(self.planner.check_joint_limit(qpos), in_limit, f"Joint limit check failed for qpos: {qpos} which should be {'in' if in_limit else 'out'} of limit")
    
  def test_pad_qpos(self):
    for _ in range(100):
      full_qpos = []
      for limit in self.planner.joint_limits:
        full_qpos.append(np.random.uniform(limit[0], limit[1]))
      non_full_qpos = []
      for limit in self.planner.joint_limits[:7]:
        non_full_qpos.append(np.random.uniform(limit[0], limit[1]))
      self.planner.robot.set_qpos(full_qpos, full=True)
      padded_qpos = self.planner.pad_qpos(non_full_qpos)
      self.planner.robot.set_qpos(non_full_qpos, full=False)
      self.assertTrue(np.allclose(padded_qpos, self.planner.robot.get_qpos(), atol=1e-2))

  def test_self_collision(self):
    self_collision_qpos = [0, 1.36, 0, -3, -3, 3, -1]
    self.assertTrue(self.planner.check_for_self_collision(self.planner.robot, self_collision_qpos))
    self_collision_qpos[0] += 1  # rotating the robot around the base should not cause self-collision to disappear
    self.assertTrue(self.planner.check_for_self_collision(self.planner.robot, self_collision_qpos))
    collision_free_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78]
    self.assertFalse(self.planner.check_for_self_collision(self.planner.robot, collision_free_qpos))

  def test_env_collision(self):
    floor = mplib.fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
    # create a collision object for the floor, with a 10cm offset in the z direction
    floor_fcl_collision_object = mplib.fcl.CollisionObject(
        floor, [0, 0, -0.1], [1, 0, 0, 0]
    )
    # update the planning world with the floor collision object
    self.planner.set_normal_object("floor", floor_fcl_collision_object)
    
    env_collision_qpos = [0, 1.5, 0, -1.5, 0, 0, 0]  # this qpos causes several joints to dip below the floor
    self.assertTrue(self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos))
    
    # remove the floor and check for env-collision returns no collision
    self.planner.remove_normal_object("floor")
    self.assertFalse(self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos))

  def test_IK(self):
    pass


if __name__ == "__main__":
  unittest.main()
