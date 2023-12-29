#!/usr/bin/env python3

import sapien.core as sapien
from sapien.utils.viewer import Viewer
import mplib
import numpy as np

class DemoSetup():
  def __init__(self, **kwargs):
    pass

  def setup_scene(self, **kwargs):
    self.engine = sapien.Engine()
    self.renderer = sapien.SapienRenderer()
    self.engine.set_renderer(self.renderer)

    scene_config = sapien.SceneConfig()
    self.scene = self.engine.create_scene(scene_config)
    self.scene.set_timestep(kwargs.get('timestep', 1 / 240))
    self.scene.add_ground(kwargs.get('ground_height', 0))
    self.scene.default_physical_material = self.scene.create_physical_material(
      kwargs.get('static_friction', 1),
      kwargs.get('dynamic_friction', 1),
      kwargs.get('restitution', 0)
    )
    self.scene.set_ambient_light(kwargs.get('ambient_light', [0.5, 0.5, 0.5]))
    shadow = kwargs.get('shadow', True)
    direction_lights = kwargs.get('direction_lights', [[[0, 1, -1], [0.5, 0.5, 0.5]]])
    for direction_light in direction_lights:
      self.scene.add_directional_light(direction_light[0], direction_light[1], shadow=shadow)
    point_lights = kwargs.get('point_lights', [[[1,2,2],[1,1,1]], [[1,-2,2],[1,1,1]], [[-1,0,1],[1,1,1]]])
    for point_light in point_lights:
      self.scene.add_point_light(point_light[0], point_light[1], shadow=shadow)

    self.viewer = Viewer(self.renderer)
    self.viewer.set_scene(self.scene)
    self.viewer.set_camera_xyz(
      x=kwargs.get('camera_xyz_x', 1.2),
      y=kwargs.get('camera_xyz_y', 0.25),
      z=kwargs.get('camera_xyz_z', 0.4)
    )
    self.viewer.set_camera_rpy(
      r=kwargs.get('camera_rpy_r', 0),
      p=kwargs.get('camera_rpy_p', -0.4),
      y=kwargs.get('camera_rpy_y', 2.7)
    )

  def load_robot(self, **kwargs):
    """ must call setup_scene() first """
    loader: sapien.URDFLoader = self.scene.create_urdf_loader()
    loader.fix_root_link = True
    self.robot: sapien.Articulation = loader.load(kwargs.get('urdf_path', "./data/panda/panda.urdf"))
    self.robot.set_root_pose(
      sapien.Pose(kwargs.get("robot_origin_xyz", [0,0,0]), kwargs.get("robot_origin_quat", [1,0,0,0]))
    )
    self.active_joints = self.robot.get_active_joints()
    for joint in self.active_joints:
      joint.set_drive_property(
        stiffness=kwargs.get('joint_stiffness', 1000),
        damping=kwargs.get('joint_damping', 200)
      )

  def setup_planner(self, **kwargs):
    self.planner = mplib.Planner(
      urdf=kwargs.get('urdf_path', "./data/panda/panda.urdf"),
      srdf=kwargs.get('srdf_path', "./data/panda/panda.srdf"),
      move_group=kwargs.get('move_group', 'panda_hand')
    )

  def follow_path(self, result):
    n_step = result['position'].shape[0]
    for i in range(n_step):
      qf = self.robot.compute_passive_force(
        external=False,
        gravity=True, 
        coriolis_and_centrifugal=True)
      self.robot.set_qf(qf)
      for j in range(len(self.planner.move_group_joint_indices)):
        self.active_joints[j].set_drive_target(result['position'][i][j])
        self.active_joints[j].set_drive_velocity_target(result['velocity'][i][j])
      self.scene.step()
      if i % 4 == 0:
        self.scene.update_render()
        self.viewer.render()

  def set_gripper(self, pos):
    for joint in self.active_joints[-2:]:
      joint.set_drive_target(pos)
    for i in range(100): 
      qf = self.robot.compute_passive_force(
        external=False,
        gravity=True, 
        coriolis_and_centrifugal=True)
      self.robot.set_qf(qf)
      self.scene.step()
      if i % 4 == 0:
        self.scene.update_render()
        self.viewer.render()

  def open_gripper(self):
    self.set_gripper(0.4)

  def close_gripper(self):
    self.set_gripper(0)

  def move_to_pose_with_RRTConnect(self, pose, use_point_cloud=False, use_attach=False):
    result = self.planner.plan_qpos_to_pose(
      pose,
      self.robot.get_qpos(),
      time_step=1/250,
      use_point_cloud=use_point_cloud,
      use_attach=use_attach,
      planner_name="RRTConnect"
    )
    if result['status'] != "Success":
      print(result['status'])
      return -1
    self.follow_path(result)
    return 0

  def move_to_pose_with_screw(self, pose, use_point_cloud=False, use_attach=False):
    result = self.planner.plan_screw(
      pose, self.robot.get_qpos(), time_step=1/250, use_point_cloud=use_point_cloud, use_attach=use_attach)
    if result['status'] == "Success":
      self.follow_path(result)
      return 0
    else:
      return self.move_to_pose_with_RRTConnect(pose, use_point_cloud, use_attach)


  def move_to_pose(self, pose, with_screw=True, use_point_cloud=False, use_attach=False):
    if with_screw:
      return self.move_to_pose_with_screw(pose, use_point_cloud, use_attach)
    else:
      return self.move_to_pose_with_RRTConnect(pose, use_point_cloud, use_attach)