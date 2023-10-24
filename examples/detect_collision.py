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
    print("\n-----self collision free check-----")
    self_collision_free_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78]
    self.print_collisions(self.planner.check_for_self_collision(self.planner.robot, self_collision_free_qpos))
    
    print("\n-----self collision check-----")
    self_collision_qpos = [0, 1.36, 0, -3, -3, 3, -1]
    self.print_collisions(self.planner.check_for_self_collision(self.planner.robot, self_collision_qpos))

    print("\n-----env collision check-----")
    floor = trimesh.creation.box([2,2,0.1])
    point_cloud, _ = trimesh.sample.sample_surface(floor, 10000)
    point_cloud += [0, 0, -0.1]
    self.planner.update_point_cloud(point_cloud)
    env_collision_qpos = [0, 1.5, 0, -1.5, 0, 0, 0]
    self.planner.planning_world.set_use_point_cloud(True)
    self.print_collisions(self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos))
    
    print("\n-----env collision causing planner to timeout-----")
    status, path = self.planner.planner.plan(start_state=self_collision_free_qpos, goal_states=[env_collision_qpos])
    print(status, path)


if __name__ == '__main__':
  demo = DetectCollisionDemo()
  demo.demo()
