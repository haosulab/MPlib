import sapien.core as sapien
from .demo_setup import DemoSetup

class PlanningDemo(DemoSetup):
    """ The shows the planner's ability to generate a collision free path with the straight path causes collisions """
    def __init__(self):
        """
        Same setup as demo.py except the boxes are of difference sizes and different use
        Red box is the target we want to grab
        Blue box is the obstacle we want to avoid
        green box is the landing pad on which we want to place the red box
        """
        super().__init__()
        self.setup_scene()
        self.load_robot()
        self.setup_planner()

        # Set initial joint positions
        init_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0]
        self.robot.set_qpos(init_qpos)

        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        self.table = builder.build_kinematic(name='table')
        self.table.set_pose(sapien.Pose([0.56, 0, - 0.025]))

        # red box is the target we want to grab
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06], color=[1, 0, 0])
        self.red_cube = builder.build(name='red_cube')
        self.red_cube.set_pose(sapien.Pose([0.7, 0, 0.06]))

        # green box is the landing pad on which we want to place the red box
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.04, 0.04, 0.005])
        builder.add_box_visual(half_size=[0.04, 0.04, 0.005], color=[0, 1, 0])
        self.green_cube = builder.build(name='green_cube')
        self.green_cube.set_pose(sapien.Pose([0.4, 0.3, 0.005]))

        # blue box is the obstacle we want to avoid
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.05, 0.2, 0.1])
        builder.add_box_visual(half_size=[0.05, 0.2, 0.1], color=[0, 0, 1])
        self.blue_cube = builder.build(name='blue_cube')
        self.blue_cube.set_pose(sapien.Pose([0.55, 0, 0.1]))

    def add_point_cloud(self):
        """ we tell the planner about the obstacle through a point cloud """
        import trimesh
        box = trimesh.creation.box([0.1, 0.4, 0.2])
        points, _ = trimesh.sample.sample_surface(box, 1000)
        points += [0.55, 0, 0.1]
        self.planner.update_point_cloud(points, radius=0.02)
        return 

    def demo(self, with_screw=True, use_point_cloud=True, use_attach=True):
        """
        We pick up the red box while avoiding the blue box and place it back down on top of the green box
        """
        pickup_pose = [0.7, 0, 0.12, 0, 1, 0, 0]
        delivery_pose = [0.4, 0.3, 0.13, 0, 1, 0, 0]
        
        # tell the planner where the obstacle is
        if use_point_cloud:
            self.add_point_cloud()
        
        # move to the pickup pose
        pickup_pose[2] += 0.2
        # no need to check collision against attached object since nothing picked up yet
        self.move_to_pose(pickup_pose, with_screw, use_point_cloud, use_attach=False) 
        self.open_gripper()
        pickup_pose[2] -= 0.12
        # no attach since nothing picked up yet
        self.move_to_pose(pickup_pose, with_screw, use_point_cloud, use_attach=False) 
        self.close_gripper()

        if use_attach:
            self.planner.update_attached_box([0.04, 0.04, 0.12], [0, 0, 0.14, 1, 0, 0, 0])

        # move to the delivery pose
        pickup_pose[2] += 0.12
        self.move_to_pose(pickup_pose, with_screw, use_point_cloud, use_attach) 
        delivery_pose[2] += 0.2
        self.move_to_pose(delivery_pose, with_screw, use_point_cloud, use_attach) 
        delivery_pose[2] -= 0.12
        self.move_to_pose(delivery_pose, with_screw, use_point_cloud, use_attach) 
        self.open_gripper()
        delivery_pose[2] += 0.12
        self.move_to_pose(delivery_pose, with_screw, use_point_cloud, use_attach=False) 

if __name__ == '__main__':
    """
    As you change some of the parameters, you will see different behaviors
    In particular, when point cloud is not used, the robot will attemt to go through the blue box
    If attach is not used, the robot will avoid the blue box on its way to the pickup pose but will knock it over with the attached red cube on its way to the delivery pose
    """
    demo = PlanningDemo()
    demo.demo(False, True, True)
