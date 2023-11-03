#!/usr/bin/env python3

import mplib
import numpy as np
import trimesh

from demo_setup import DemoSetup

class DetectCollisionDemo(DemoSetup):
  def __init__(self):
    super().__init__()
    self.setup_planner()

  def print_collisions(self, collisions):
    for collision in collisions:
      print(f"{collision.link_name1} of object {collision.object_name1} collides with {collision.link_name2} of object {collision.object_name2}")

  def demo(self):
    print("\n----- self-collision-free pose -----")
    # if the joint pose does not include the gripper joints, it will be set to the current gripper joint angle
    self_collision_free_qpos = [0, 0, 0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78]
    self.print_collisions(self.planner.check_for_self_collision(self.planner.robot, self_collision_free_qpos))
    
    print("\n----- self-collision pose -----")
    self_collision_qpos = [0, 0, 0, 1.36, 0, -3, -3, 3, -1]
    self.print_collisions(self.planner.check_for_self_collision(self.planner.robot, self_collision_qpos))

    print("\n----- env-collision pose -----")
    floor = trimesh.creation.box([2,2,0.1])  # create a 2x2m floor with thickness 0.1m
    point_cloud, _ = trimesh.sample.sample_surface(floor, 10000)
    point_cloud += [0, 0, -0.1]  # shift the entire point cloud down by 0.1m so we do not collide with the base
    self.planner.update_point_cloud(point_cloud)
    env_collision_qpos = [0, 0, 0, 1.5, 0, -1.5, 0, 0, 0]  # this pose causes several joints to dip below the floor

    # the planner will ignore the point cloud if not enabled
    self.planner.planning_world.set_use_point_cloud(True)
    # additionally, you might want to enable attach to treat whatever the gripper is grabbing as part of the robot
    # self.planner.planning_world.set_use_attach(True)
    self.print_collisions(self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos))
    
    print("\n----- env-collision causing planner to timeout -----")
    status, path = self.planner.planner.plan(start_state=self_collision_free_qpos, goal_states=[env_collision_qpos])
    print(status, path)


if __name__ == '__main__':
  demo = DetectCollisionDemo()
  demo.demo()
