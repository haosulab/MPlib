#!/usr/bin/env python3

import mplib
import numpy as np
import trimesh

from demo_setup import DemoSetup

class ConstrainedPlanningDemo(DemoSetup):
  def __init__(self):
    super().__init__()
    self.setup_planner(constrained_problem=True)

  def demo(self):
    starting_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78]
    self.planner.robot.set_qpos(starting_qpos)
    poses = [[0.4, 0.3, 0.12, 0, 1, 0, 0],
             [0.2, -0.3, 0.08, 0, 1, 0, 0],
             [0.6, 0.1, 0.14, 0, 1, 0, 0]]
    for i in range(3):
      result = self.planner.plan(
        poses[i],
        self.planner.robot.get_qpos(),
        time_step=1/250,
        use_point_cloud=False,
        use_attach=False,
        planner_name="RRTConnect"
      )
      print(result)

if __name__ == '__main__':
  demo = ConstrainedPlanningDemo()
  demo.demo()
