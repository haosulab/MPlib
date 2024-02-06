import os
import numpy as np
import unittest
from transforms3d.quaternions import mat2quat, quat2mat
from mplib import Planner
import mplib
from mplib.pymp.collision_detection import fcl
import trimesh

FILE_ABS_DIR = os.path.dirname(os.path.abspath(__file__))

PANDA_SPEC = {
  "urdf": f"{FILE_ABS_DIR}/../data/panda/panda.urdf",
  "srdf": f"{FILE_ABS_DIR}/../data/panda/panda.srdf",
  "move_group": "panda_hand",
}

class TestPlannerSimple(unittest.TestCase):
  def setUp(self):
    self.planner = Planner(**PANDA_SPEC)
    self.target_pose = [0.4, 0.3, 0.12, 0, 1, 0, 0]
    self.init_qpos = np.array([0, 0.2, 0, -2.6, 0, 3.0, 0.8, 0, 0])
    self.joint_limits = [[-2.8973, 2.8973],
               [-1.7628, 1.7628],
               [-2.8973, 2.8973],
               [-3.0718, -0.0698],
               [-2.8973, 2.8973],
               [-0.0175, 3.7525],
               [-2.8973, 2.8973],
               [0, 0.04],
               [0, 0.04]]

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
    result_sampling = self.planner.plan_qpos_to_pose(self.target_pose, self.init_qpos)
    self.assertEqual(result_sampling["status"], "Success")
    last_qpos = result_sampling["position"][-1]
    self.planner.robot.set_qpos(last_qpos)
    self.assertTrue(np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-2))

  def test_planning_to_qpos(self):
    num_success = 0
    expected_least_num_success = 5
    for _ in range(10):
      target_qpos = self.sample_qpos()[:7]
      result_sampling = self.planner.plan_qpos_to_qpos([target_qpos], self.init_qpos)
      # might not always be a valid pose, so check if the planner succeeded
      if result_sampling["status"] == "Success":
        num_success += 1
        last_qpos = result_sampling["position"][-1]
        self.planner.robot.set_qpos(last_qpos)
        self.assertTrue(np.allclose(last_qpos, target_qpos, atol=1e-2))
    self.assertGreaterEqual(num_success, expected_least_num_success)

  def test_planning_screw(self):
    result_sampling = self.planner.plan_screw(self.target_pose, self.init_qpos)
    self.assertEqual(result_sampling["status"], "Success")
    last_qpos = result_sampling["position"][-1]
    self.planner.robot.set_qpos(last_qpos)
    self.assertTrue(np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-2))

  def test_check_joint_limit(self, tolerance=2e-2):
    for _ in range(100):
      qpos = np.random.uniform(-np.pi, np.pi, size=7)
      in_limit = True
      for joint_angle, limit in zip(qpos, self.joint_limits):
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
      full_qpos = self.sample_qpos()
      non_full_qpos = self.sample_qpos()[:7]
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
    floor = fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
    # create a collision object for the floor, with a 10cm offset in the z direction
    floor_fcl_collision_object = fcl.CollisionObject(
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
    num_success = 0
    expected_least_num_success = 10
    for _ in range(100):
      # generate some random poses that should be within the robot's workspace
      pose = self.sample_pose()
      status, results = self.planner.IK(pose, self.init_qpos)
      num_success += status == "Success"
      if status == "Success":
        for result_qpos in results:
          self.planner.robot.set_qpos(result_qpos, full=True)
          self.assertTrue(np.allclose(self.get_end_effector_pose(), pose, atol=1e-2))
    self.assertGreaterEqual(num_success, expected_least_num_success)

    # now put down a floor and check that the robot can't reach the pose
    floor = fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
    # create a collision object for the floor, with a 10cm offset in the z direction
    floor_fcl_collision_object = fcl.CollisionObject(
        floor, [0, 0, -0.1], [1, 0, 0, 0]
    )
    status, _ = self.planner.IK([0.4,0.3,-0.1,0,1,0,0], self.init_qpos)
    self.assertEqual(status, "Success")
    self.planner.set_normal_object("floor", floor_fcl_collision_object)
    status, _ = self.planner.IK([0.4,0.3,-0.1,0,1,0,0], self.init_qpos)
    self.assertNotEqual(status, "Success")

  def test_set_base_pose(self):
    target_pose_tf = np.eye(4)
    target_pose_tf[:3, 3] = self.target_pose[:3]
    target_pose_tf[:3, :3] = quat2mat(self.target_pose[3:])

    for _ in range(10):
      base_pose = self.sample_pose()
      self.planner.set_base_pose(base_pose)

      quat = base_pose[3:]
      hom_mat = np.eye(4)
      hom_mat[:3, :3] = quat2mat(quat)
      hom_mat[:3, 3] = base_pose[:3]
      transformed_target_tf = hom_mat @ target_pose_tf
      transformed_target_pose = np.concatenate([transformed_target_tf[:3, 3], mat2quat(transformed_target_tf[:3, :3])])
      
      result_sampling = self.planner.plan_qpos_to_pose(transformed_target_pose, self.init_qpos)
      self.assertEqual(result_sampling["status"], "Success")
      last_qpos_sampling = result_sampling["position"][-1]
      self.planner.robot.set_qpos(last_qpos_sampling)
      self.assertTrue(np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-2))

      result_screw = self.planner.plan_screw(transformed_target_pose, self.init_qpos)
      self.assertEqual(result_screw["status"], "Success")
      last_qpos_screw = result_screw["position"][-1]
      self.planner.robot.set_qpos(last_qpos_screw)
      self.assertTrue(np.allclose(self.get_end_effector_pose(), self.target_pose, atol=1e-2))
      
      result_sampling = self.planner.plan_qpos_to_pose(self.target_pose, self.init_qpos, wrt_world=False)
      self.assertEqual(result_sampling["status"], "Success")
      last_qpos_sampling = result_sampling["position"][-1]
      self.planner.robot.set_qpos(last_qpos_sampling)

  def add_point_cloud(self):
    box = trimesh.creation.box([0.1, 0.4, 0.2])
    points, _ = trimesh.sample.sample_surface(box, 1000)
    points += [0.55, 0, 0.1]
    self.planner.update_point_cloud(points, radius=0.02)

  def test_update_point_cloud(self):
    # use screw based planning. first succeeds but after point cloud obstacle fails
    pose = [0.7, 0, 0.12, 0, 1, 0, 0]
    self.add_point_cloud()
    result_screw = self.planner.plan_screw(pose, self.init_qpos)
    self.assertEqual(result_screw["status"], "Success")
    result_screw = self.planner.plan_screw(pose, self.init_qpos, use_point_cloud=True)
    self.assertNotEqual(result_screw["status"], "Success")

  def test_update_attach(self):
    starting_qpos = [0, 0.48, 0, -1.48, 0, 1.96, 0.78]
    target_pose = [0.4, 0.3, 0.33, 0, 1, 0, 0]
    self.planner.update_attached_box([0.04, 0.04, 0.12], [0, 0, 0.14, 1, 0, 0, 0])
    self.add_point_cloud()

    result_screw = self.planner.plan_screw(target_pose, starting_qpos, use_point_cloud=True)
    self.assertEqual(result_screw["status"], "Success")
    result_screw = self.planner.plan_screw(target_pose, starting_qpos, use_point_cloud=True, use_attach=True)
    self.assertNotEqual(result_screw["status"], "Success")

if __name__ == "__main__":
  unittest.main()
