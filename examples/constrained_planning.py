#!/usr/bin/env python3

import mplib
import numpy as np

from demo_setup import DemoSetup

class ConstrainedPlanningDemo(DemoSetup):
  def __init__(self):
    super().__init__()
    self.setup_scene()
    self.load_robot()
    self.setup_planner(constrained_problem=True)

  def demo(self):
    starting_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0]
    self.robot.set_qpos(starting_qpos)
    self.planner.robot.set_qpos(starting_qpos[:7])
    poses = [[0.4, 0.3, 0.22, 0, 1, 0, 0],
             [0.2, -0.3, 0.18, 0, 1, 0, 0],
             [0.6, 0.1, 0.24, 0, 1, 0, 0]]
    for i in range(3):
      self.move_to_pose_with_RRTConnect(poses[i])

if __name__ == '__main__':
  demo = ConstrainedPlanningDemo()
  demo.demo()
