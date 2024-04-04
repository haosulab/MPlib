import sapien.core as sapien

from mplib.examples.demo_setup import DemoSetup


class PlanningDemo(DemoSetup):
    """
    This is the most basic demo of the motion planning library where the robot tries to
    shuffle three boxes around.
    """

    def __init__(self):
        """
        Setting up the scene, the planner, and adding some objects to the scene.
        Afterwards, put down a table and three boxes.
        For details on how to do this, see the sapien documentation.
        """
        super().__init__()
        # load the world, the robot, and then setup the planner.
        # See demo_setup.py for more details
        self.setup_scene()
        self.load_robot()
        self.setup_planner()

        # Set initial joint positions
        init_qpos = [0, 0.19, 0.0, -2.62, 0.0, 2.94, 0.79, 0, 0]
        self.robot.set_qpos(init_qpos)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        table = builder.build_kinematic(name="table")
        table.set_pose(sapien.Pose([0.56, 0, -0.025]))

        # boxes ankor
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06])
        red_cube = builder.build(name="red_cube")
        red_cube.set_pose(sapien.Pose([0.4, 0.3, 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.04])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.04])
        green_cube = builder.build(name="green_cube")
        green_cube.set_pose(sapien.Pose([0.2, -0.3, 0.04]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.07])
        blue_cube = builder.build(name="blue_cube")
        blue_cube.set_pose(sapien.Pose([0.6, 0.1, 0.07]))
        # boxes ankor end

    def demo(self):
        """
        Declare three poses for the robot to move to, each one corresponding to
        the position of a box.
        Pick up the box, and set it down 0.1m to the right of its original position.
        """
        # target poses ankor
        poses = [
            [0.4, 0.3, 0.12, 0, 1, 0, 0],
            [0.2, -0.3, 0.08, 0, 1, 0, 0],
            [0.6, 0.1, 0.14, 0, 1, 0, 0],
        ]
        # target poses ankor end
        # execute motion ankor
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
        # execute motion ankor end


if __name__ == "__main__":
    demo = PlanningDemo()
    demo.demo()
