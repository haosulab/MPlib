# NOTE: this demo requires sapien 3 to run

import mplib
import numpy as np
import sapien
import sapien.physx as physx
from mplib.pymp.fcl import Box, Capsule, CollisionObject, Convex, Cylinder, Plane, Sphere
from sapien import Entity, Pose
from transforms3d.euler import euler2quat
from transforms3d.quaternions import quat2mat

from demo_setup import DemoSetup

def add_normal_object(planner: mplib.Planner, entity: Entity) -> None:
    """Adds a normal_object to planner from entity"""
    component = entity.find_component_by_type(physx.PhysxRigidBaseComponent)
    assert component is not None, \
        f"No PhysxRigidBaseComponent found in {entity.name}: {entity.components=}"
    pose: Pose = entity.pose

    # Entity should only have 1 collision shape
    assert len(component.collision_shapes) == 1, \
        f"Should only have 1 collision shape, got {component.collision_shapes=}"
    col_shape = component.collision_shapes[0]

    if isinstance(col_shape, physx.PhysxCollisionShapeBox):
        col_geometry = Box(side=col_shape.half_size * 2)
    elif isinstance(col_shape, physx.PhysxCollisionShapeCapsule):
        col_geometry = Capsule(
            radius=col_shape.radius, lz=col_shape.half_length * 2
        )
        # NOTE: physx Capsule has x-axis along capsule height
        # FCL Capsule has z-axis along capsule height
        pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))
    elif isinstance(col_shape, physx.PhysxCollisionShapeConvexMesh):
        assert np.allclose(col_shape.scale, 1.0), \
            f"Not unit scale {col_shape.scale}, need to rescale vertices?"
        col_geometry = Convex(
            vertices=col_shape.vertices, faces=col_shape.triangles
        )
    elif isinstance(col_shape, physx.PhysxCollisionShapeCylinder):
        col_geometry = Cylinder(
            radius=col_shape.radius, lz=col_shape.half_length * 2
        )
        # NOTE: physx Cylinder has x-axis along cylinder height
        # FCL Cylinder has z-axis along cylinder height
        pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))
    elif isinstance(col_shape, physx.PhysxCollisionShapePlane):
        normal = quat2mat(pose.q)[:3, 2]
        offset = -normal.dot(pose.p)
        # NOTE: fcl Plane causes segfault so using a box to approximate
        # col_geometry = Plane(normal=normal, offset=offset)
        col_geometry = Box(side=[5, 5, 0.01])
        print(f"WARNING: Using a box to approximate the plane geometry {entity.name} since Plane is not supported.")
        # also need to move the box in the opposite direction of the normal to avoid collision
        pose = pose * Pose(p=-normal*0.01)
    elif isinstance(col_shape, physx.PhysxCollisionShapeSphere):
        col_geometry = Sphere(radius=col_shape.radius)
    elif isinstance(col_shape, physx.PhysxCollisionShapeTriangleMesh):
        # NOTE: see mplib.pymp.fcl.Triangle
        raise NotImplementedError(
            "Support for TriangleMesh collision is not implemented yet."
        )
    else:
        raise TypeError(f"Unknown shape type: {type(col_shape)}")
    col_obj = CollisionObject(col_geometry, pose.p, pose.q)
    planner.planning_world.set_normal_object(entity.name, col_obj)

def inject_collision_objects(planner: mplib.Planner, scene: sapien.Scene) -> None:
    """Injects collision objects from planning_world to scene"""
    non_articulated_objects = scene.get_all_actors()

    for entity in non_articulated_objects:
        print(f"Adding {entity.name} to planning world")
        add_normal_object(planner, entity)

def update_object_pose(planner: mplib.Planner, scene: sapien.Scene) -> None:
    """Updates planner objects pose (w/o robot) with current environment state"""
    non_articulated_objects = scene.get_all_actors()

    for entity in non_articulated_objects:
        name = entity.name
        col_obj = planner.planning_world.get_normal_object(name)
        if not col_obj:
            add_normal_object(planner, entity)
        else:
            pose: Pose = entity.pose
            # NOTE: Convert poses for Capsule/Cylinder
            if isinstance(col_obj.get_collision_geometry(), (Capsule, Cylinder)):
                pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))
            col_obj.set_transformation(np.hstack((pose.p, pose.q)))

class Sapien2PlannerDemo(DemoSetup):
    def __init__(self):
        super().__init__()
        self.setup_scene()
        self.load_robot()
        self.setup_planner()

        # Set initial joint positions
        self.init_qpos = [0, 0.19, 0.0, -2.61, 0.0, 2.94, 0.78, 0, 0]
        self.robot.set_qpos(self.init_qpos)

    def add_collision_objects(self):
        # table top
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.4, 0.4, 0.025])
        builder.add_box_visual(half_size=[0.4, 0.4, 0.025])
        self.table = builder.build_kinematic(name='table')
        self.table.set_pose(sapien.Pose([0.56, 0, -0.025]))

        # boxes
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.06])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.06])
        self.red_cube = builder.build(name='red_cube')
        self.red_cube.set_pose(sapien.Pose([0.4, 0.3, 0.06]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.04])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.04])
        self.green_cube = builder.build(name='green_cube')
        self.green_cube.set_pose(sapien.Pose([0.2, -0.3, 0.04]))

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.02, 0.02, 0.07])
        builder.add_box_visual(half_size=[0.02, 0.02, 0.07])
        self.blue_cube = builder.build(name='blue_cube')
        self.blue_cube.set_pose(sapien.Pose([0.6, 0.1, 0.07]))

    def demo(self):
        poses = [[0.4, 0.3, 0.22, 0, 1, 0, 0],
                [0.2, -0.3, 0.18, 0, 1, 0, 0],
                [0.6, 0.1, 0.24, 0, 1, 0, 0]]
        
        print("Without collision objects")
        for i in range(3):
            pose = poses[i]
            print(f"Moving to pose {pose}")
            self.move_to_pose_with_RRTConnect(pose)  # no problem

        # Add objects to planning world
        self.add_collision_objects()
        inject_collision_objects(self.planner, self.scene)

        print("With collision objects")
        for i in range(3):
            pose = poses[i]
            print(f"Moving to pose {pose}")
            self.move_to_pose_with_RRTConnect(pose)  # fails due to collision with the added objs

if __name__ == '__main__':
    demo = Sapien2PlannerDemo()
    demo.demo()
