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
    if len(collisions) == 0:
      print("No collision")
      return
    for collision in collisions:
      print(f"{collision.link_name1} of entity {collision.object_name1} collides with "
            f"{collision.link_name2} of entity {collision.object_name2}")

  def demo(self):
    floor = mplib.planner.fcl.Box([2,2,0.1])  # create a 2 x 2 x 0.1m box
    # create a collision object for the floor, with a 10cm offset in the z direction
    floor_fcl_collision_object = mplib.planner.fcl.CollisionObject(floor, [0,0,-0.1], [1,0,0,0])
    # update the planning world with the floor collision object
    self.planner.set_normal_object("floor", floor_fcl_collision_object)

    print("\n----- self-collision-free pose -----")
    # if the joint pose does not include the gripper joints, it will be set to the current gripper joint angle
    self_collision_free_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78]
    self.print_collisions(self.planner.check_for_self_collision(self.planner.robot, self_collision_free_qpos))
    
    print("\n----- self-collision pose -----")
    self_collision_qpos = [0, 1.36, 0, -3, -3, 3, -1]
    self.print_collisions(self.planner.check_for_self_collision(self.planner.robot, self_collision_qpos))

    print("\n----- env-collision-free pose -----")
    env_collision_free_qpos = self_collision_free_qpos
    self.print_collisions(self.planner.check_for_env_collision(self.planner.robot, env_collision_free_qpos))

    print("\n----- env-collision pose -----")
    env_collision_qpos = [0, 1.5, 0, -1.5, 0, 0, 0]  # this pose causes several joints to dip below the floor
    self.print_collisions(self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos))
    
    print("\n----- env-collision causing planner to timeout -----")
    status, path = self.planner.planner.plan(start_state=self_collision_free_qpos, goal_states=[env_collision_qpos])
    print(status, path)

    print("\n----- no more env-collision after removing the floor -----")
    self.planner.remove_normal_object("floor")
    self.print_collisions(self.planner.check_for_env_collision(self.planner.robot, env_collision_qpos))

if __name__ == '__main__':
  demo = DetectCollisionDemo()
  demo.demo()
