#!/usr/bin/env python3

import mplib
import numpy as np

from demo_setup import DemoSetup

class ConstrainedPlanningDemo(DemoSetup):
  def __init__(self):
    super().__init__()
    self.setup_scene()
    self.load_robot()
    self.setup_planner()

  def add_point_cloud(self):
    import trimesh
    box = trimesh.creation.box([0.1, 0.4, 0.2])
    points, _ = trimesh.sample.sample_surface(box, 1000)
    all_pts = np.concatenate([points+[-0.65, -0.1, 0.4], points+[0.55, 0, 0.1]], axis=0)
    self.planner.update_point_cloud(all_pts, radius=0.02)
    return

  def demo(self):
    starting_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0]
    self.robot.set_qpos(starting_qpos)
    self.planner.robot.set_qpos(starting_qpos[:7])
    poses = [[-0.4, -0.3, 0.28, 0.0000563, -0.0707372, -0.9974947, -0.0007943],
             [0.6, 0.1, 0.44, 0.0006988, -0.8775823, -0.4794254, -0.0003818],
             [0, -0.3, 0.5, 0, 1, 0, 0]]
    
    self.add_point_cloud()

    # with constraint
    print("with constraint")
    for pose in poses:
      result = self.planner.plan(
        pose,
        self.robot.get_qpos(),
        time_step=1/250,
        use_point_cloud=True,
        use_attach=False,
        planner_name="RRTConnect",
        align_axis=[0.0,0.0,-1.0]
      )
      if result['status'] != "Success":
        print(result['status'])
        return -1
      self.follow_path(result)

    # without constraint
    print("without constraint")
    for pose in poses:
      result = self.planner.plan(
        pose,
        self.robot.get_qpos(),
        time_step=1/250,
        use_point_cloud=True,
        use_attach=False,
        planner_name="RRTConnect",
        no_simplification=True
      )
      if result['status'] != "Success":
        print(result['status'])
        return -1
      self.follow_path(result)


if __name__ == '__main__':
  demo = ConstrainedPlanningDemo()
  demo.demo()
