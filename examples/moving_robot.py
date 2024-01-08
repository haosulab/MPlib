import sapien.core as sapien
from demo_setup import DemoSetup

class PlanningDemo(DemoSetup):
    """ This is identical to demo.py except the whole scene is shifted to the bottom right by 1 meter respectively """
    def __init__(self):
        """
        Setting up the scene, the planner, and adding some objects to the scene
        Note that we place the robot at [1,1,0] in the simulation and also tell the planner that the robot is at [1,1,0]
        Afterwards, put down a table and three boxes. For details on how to do this, see the sapien documentation
        Compared to demo.py, all the props are shifted by 1 meter in the x and y direction
        """
        super().__init__()
        self.setup_scene()
        self.load_robot(robot_origin_xyz=[1, 1, 0])
        self.setup_planner()

        # We also need to tell the planner where the base is since the sim and planner don't share info
        self.planner.set_base_pose([1, 1, 0, 1, 0, 0, 0])

        # Set initial joint positions
        init_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0]
        self.robot.set_qpos(init_qpos)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        self.table = builder.build_kinematic(name='table')
        self.table.set_pose(sapien.Pose([1.56, 1, -0.025]))

        # boxes
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], color=[1, 0, 0])
        self.red_cube = builder.build(name='red_cube')
        self.red_cube.set_pose(sapien.Pose([1.4, 1.3, 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.04])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.04], color=[0, 1, 0])
        self.green_cube = builder.build(name='green_cube')
        self.green_cube.set_pose(sapien.Pose([1.2, 0.7, 0.04]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.07], color=[0, 0, 1])
        self.blue_cube = builder.build(name='blue_cube')
        self.blue_cube.set_pose(sapien.Pose([1.6, 1.1, 0.07]))

    def demo(self):
        """
        Same demo as demo.py.
        Although we shifted everything, the poses have not changed because these are w.r.t. the robot base
        Alternatively, we can also shift the poses by 1 meter in the x and y direction and tell the planner
        the poses are specified with respect to the world
        """
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

        # convert the poses to be w.r.t. the world
        for pose in poses:
            pose[0] += 1
            pose[1] += 1

        # plan a path to the first pose
        result = self.planner.plan_qpos_to_pose(
            poses[0],
            self.planner.robot.get_qpos(),
            time_step=1/250,
            wrt_world=True
        )
        if result['status'] != "Success":
            print(result['status'])
            return -1
        self.follow_path(result)

if __name__ == '__main__':
    """ Driver code """
    demo = PlanningDemo()
    demo.demo()
