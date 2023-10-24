import sapien.core as sapien
import mplib
import numpy as np
from demo_setup import DemoSetup

class PlanningDemo(DemoSetup):
    def __init__(self):
        super().__init__()
        self.setup_scene()
        self.load_robot()
        self.setup_planner()

        # Set initial joint positions
        init_qpos = [0, 0.19634954084936207, 0.0, -2.617993877991494, 0.0, 2.941592653589793, 0.7853981633974483, 0, 0]
        self.robot.set_qpos(init_qpos)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        self.table = builder.build_kinematic(name='table')
        self.table.set_pose(sapien.Pose([0.56, 0, - 0.025]))

        # boxes
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], color=[1, 0, 0])
        self.red_cube = builder.build(name='red_cube')
        self.red_cube.set_pose(sapien.Pose([0.4, 0.3, 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.04])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.04], color=[0, 1, 0])
        self.green_cube = builder.build(name='green_cube')
        self.green_cube.set_pose(sapien.Pose([0.2, -0.3, 0.04]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.07], color=[0, 0, 1])
        self.blue_cube = builder.build(name='blue_cube')
        self.blue_cube.set_pose(sapien.Pose([0.6, 0.1, 0.07]))

    def demo(self):
        poses = [[0.4, 0.3, 0.12, 0, 1, 0, 0],
                [0.2, -0.3, 0.08, 0, 1, 0, 0],
                [0.6, 0.1, 0.14, 0, 1, 0, 0]]
        for i in range(3):
            pose = poses[i]
            pose[2] += 0.2
            self.move_to_pose(pose)
            self.open_gripper()
            pose[2] -= 0.12
            self.move_to_pose(pose)
            self.close_gripper()
            pose[2] += 0.12
            self.move_to_pose(pose)
            pose[0] += 0.1
            self.move_to_pose(pose)
            pose[2] -= 0.12
            self.move_to_pose(pose)
            self.open_gripper()
            pose[2] += 0.12
            self.move_to_pose(pose)

if __name__ == '__main__':
    demo = PlanningDemo()
    demo.demo()
