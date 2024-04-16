#!/usr/bin/env python3

from mplib import Pose
from mplib.collision_detection import fcl
from mplib.examples.demo_setup import DemoSetup


class DetectCollisionDemo(DemoSetup):
    """
    This demonstrates some of the collision detection functions in the planner.
    """

    def __init__(self):
        """Only the planner is needed this time. No simulation env required"""
        super().__init__()
        self.setup_planner()

    # print_collision ankor
    def print_collisions(self, collisions):
        """Helper function to abstract away the printing of collisions"""
        if len(collisions) == 0:
            print("No collision")
            return
        for collision in collisions:
            print(
                f"{collision.link_name1} of entity {collision.object_name1} collides"
                f" with {collision.link_name2} of entity {collision.object_name2}"
            )

    # print_collision ankor end
    def demo(self):
        """
        We test several configurations:
        1. Set robot to a self-collision-free qpos and check for self-collision returns
           no collision
        2. Set robot to a self-collision qpos and check for self-collision returns
           a collision
        3. Set robot to a env-collision-free qpos and check for env-collision returns
           no collision
        4. Set robot to a env-collision qpos and check for env-collision returns
           a collision
        5. Attempts to plan a path to a qpos is in collision with the world.
           This will cause the planner to timeout
        6. Remove the floor and check for env-collision returns no collision
        """
        # floor ankor
        floor = fcl.Box([2, 2, 0.1])  # create a 2 x 2 x 0.1m box
        # create a collision object for the floor, with a 10cm offset in the z direction
        floor_fcl_collision_object = fcl.CollisionObject(floor, Pose(p=[0, 0, -0.1]))
        # update the planning world with the floor collision object
        self.planner.planning_world.add_object("floor", floor_fcl_collision_object)
        # floor ankor end
        print("\n----- self-collision-free qpos -----")
        # if the joint qpos does not include the gripper joints,
        # it will be set to the current gripper joint angle
        self_collision_free_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78]
        self.print_collisions(
            self.planner.check_for_self_collision(self_collision_free_qpos)
        )

        print("\n----- self-collision qpos -----")
        self_collision_qpos = [0, 1.36, 0, -3, -3, 3, -1]
        self.print_collisions(
            self.planner.check_for_self_collision(self_collision_qpos)
        )

        print("\n----- env-collision-free qpos -----")
        env_collision_free_qpos = self_collision_free_qpos
        self.print_collisions(
            self.planner.check_for_env_collision(env_collision_free_qpos)
        )

        print("\n----- env-collision qpos -----")
        # this qpos causes several joints to dip below the floor
        env_collision_qpos = [0, 1.5, 0, -1.5, 0, 0, 0]
        self.print_collisions(self.planner.check_for_env_collision(env_collision_qpos))

        print("\n----- env-collision causing planner to timeout -----")
        status, path = self.planner.planner.plan(
            start_state=self_collision_free_qpos, goal_states=[env_collision_qpos]
        )
        print(status, path)

        print("\n----- no more env-collision after removing the floor -----")
        self.planner.remove_normal_object("floor")
        self.print_collisions(self.planner.check_for_env_collision(env_collision_qpos))
        # end ankor


if __name__ == "__main__":
    """Driver code"""
    demo = DetectCollisionDemo()
    demo.demo()
