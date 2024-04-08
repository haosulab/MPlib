from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
from sapien import Entity, Pose, Scene
from sapien.physx import (
    PhysxArticulation,
    PhysxArticulationLinkComponent,
    PhysxCollisionShapeBox,
    PhysxCollisionShapeCapsule,
    PhysxCollisionShapeConvexMesh,
    PhysxCollisionShapeCylinder,
    PhysxCollisionShapePlane,
    PhysxCollisionShapeSphere,
    PhysxCollisionShapeTriangleMesh,
    PhysxRigidBaseComponent,
)
from transforms3d.euler import euler2quat

from ..planner import Planner
from ..pymp import ArticulatedModel, PlanningWorld
from ..pymp.collision_detection.fcl import (
    Box,
    BVHModel,
    Capsule,
    CollisionObject,
    Convex,
    Cylinder,
    Halfspace,
    Sphere,
    collide,
    distance,
)
from ..pymp.planning import ompl
from .srdf_exporter import export_srdf
from .urdf_exporter import export_kinematic_chain_urdf


class SapienPlanningWorld(PlanningWorld):
    def __init__(
        self, sim_scene: Scene, planned_articulations: list[PhysxArticulation] = []
    ):
        """
        Creates an mplib.pymp.planning_world.PlanningWorld from a sapien.Scene.

        :param planned_articulations: list of planned articulations.
        """
        super().__init__([], [])
        self._sim_scene = sim_scene

        articulations: list[PhysxArticulation] = sim_scene.get_all_articulations()
        actors: list[Entity] = sim_scene.get_all_actors()

        for articulation in articulations:
            urdf_str = export_kinematic_chain_urdf(articulation)
            srdf_str = export_srdf(articulation)

            # Get all links with collision shapes at local_pose
            collision_links = []  # [(link_name, [CollisionObject, ...]), ...]
            for link in articulation.links:
                col_objs = self.convert_sapien_col_shape(link)
                if len(col_objs) > 0:
                    collision_links.append((link.name, col_objs))

            articulated_model = ArticulatedModel.create_from_urdf_string(
                urdf_str,
                srdf_str,
                collision_links=collision_links,
                gravity=sim_scene.get_physx_system().config.gravity,  # type: ignore
                link_names=[link.name for link in articulation.links],
                joint_names=[j.name for j in articulation.active_joints],
                verbose=False,
            )
            articulated_model.set_qpos(articulation.qpos)  # update qpos  # type: ignore

            self.add_articulation(self.get_object_name(articulation), articulated_model)

        for articulation in planned_articulations:
            self.set_articulation_planned(self.get_object_name(articulation), True)

        for entity in actors:
            component = entity.find_component_by_type(PhysxRigidBaseComponent)
            assert (
                component is not None
            ), f"No PhysxRigidBaseComponent found in {entity.name}: {entity.components=}"
            assert not isinstance(
                component, PhysxArticulationLinkComponent
            ), f"Component should not be PhysxArticulationLinkComponent: {component=}"

            # Convert collision shapes at current global pose
            col_objs = self.convert_sapien_col_shape(component)  # type: ignore
            # TODO: multiple collision shapes
            assert len(col_objs) == 1, (
                f"Should only have 1 collision object, got {len(col_objs)} shapes for "
                f"entity '{entity.name}'"
            )

            self.add_normal_object(self.get_object_name(entity), col_objs[0])

    def update_from_simulation(self, *, update_attached_object: bool = True) -> None:
        """
        Updates planning_world articulations/objects pose with current Scene state

        :param update_attached_object: whether to update the attached pose of
            all attached objects
        """
        for articulation in self._sim_scene.get_all_articulations():
            # set_qpos to update poses
            self.get_articulation(self.get_object_name(articulation)).set_qpos(
                articulation.qpos  # type: ignore
            )

        for entity in self._sim_scene.get_all_actors():
            component = entity.find_component_by_type(PhysxRigidBaseComponent)
            assert (
                component is not None
            ), f"No PhysxRigidBaseComponent found in {entity.name}: {entity.components=}"
            assert not isinstance(
                component, PhysxArticulationLinkComponent
            ), f"Component should not be PhysxArticulationLinkComponent: {component=}"

            shapes = component.collision_shapes  # type: ignore
            # TODO: multiple collision shapes
            assert len(shapes) == 1, (
                f"Should only have 1 collision shape, got {len(shapes)} shapes for "
                f"entity '{entity.name}'"
            )
            shape = shapes[0]

            pose: Pose = entity.pose * shape.local_pose
            # NOTE: Convert poses for Capsule/Cylinder
            if isinstance(
                shape, (PhysxCollisionShapeCapsule, PhysxCollisionShapeCylinder)
            ):
                pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))  # type: ignore

            # handle attached object
            if self.is_normal_object_attached(self.get_object_name(entity)):
                attached_body = self.get_attached_object(self.get_object_name(entity))
                if update_attached_object:
                    parent_posevec = (
                        attached_body.get_attached_articulation()
                        .get_pinocchio_model()
                        .get_link_pose(attached_body.get_attached_link_id())
                    )
                    parent_pose = Pose(parent_posevec[:3], parent_posevec[3:])  # type: ignore
                    pose = parent_pose.inv() * pose  # new attached pose
                    attached_body.set_pose(np.hstack((pose.p, pose.q)))
                attached_body.update_pose()
            else:
                self.get_normal_object(self.get_object_name(entity)).set_transformation(
                    np.hstack((pose.p, pose.q))
                )

    @staticmethod
    def get_object_name(obj: PhysxArticulation | Entity) -> str:
        """
        Constructs a unique name for the corresponding mplib object.
        This is necessary because mplib objects assume unique names.

        :param obj: a SAPIEN object
        :return: the unique mplib object name
        """
        if isinstance(obj, PhysxArticulation):
            return f"{obj.name}_{obj.root.entity.per_scene_id}"
        elif isinstance(obj, Entity):
            return f"{obj.name}_{obj.per_scene_id}"
        else:
            raise NotImplementedError(f"Unknown SAPIEN object type {type(obj)}")

    @staticmethod
    def convert_sapien_col_shape(
        component: PhysxRigidBaseComponent,
    ) -> list[CollisionObject]:
        """Converts a SAPIEN physx.PhysxRigidBaseComponent to an FCL CollisionObject
        Returns a list of collision_obj at their current poses.

        If the component is an articulation link, the returned collision_obj is at
        the shape's local_pose.
        Otherwise, the returned collision_obj is at the entity's global pose
        """
        shapes = component.collision_shapes
        if len(shapes) == 0:
            return []

        # NOTE: MPlib currently only supports 1 collision shape per object
        # TODO: multiple collision shapes
        assert len(shapes) == 1, (
            f"Should only have 1 collision shape, got {len(shapes)} shapes for "
            f"entity '{component.entity.name}'"
        )

        shape = shapes[0]
        if isinstance(component, PhysxArticulationLinkComponent):  # articulation link
            pose = shape.local_pose
        else:
            pose = component.entity.pose * shape.local_pose

        if isinstance(shape, PhysxCollisionShapeBox):
            collision_geom = Box(side=shape.half_size * 2)
        elif isinstance(shape, PhysxCollisionShapeCapsule):
            collision_geom = Capsule(radius=shape.radius, lz=shape.half_length * 2)
            # NOTE: physx Capsule has x-axis along capsule height
            # FCL Capsule has z-axis along capsule height
            pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))  # type: ignore
        elif isinstance(shape, PhysxCollisionShapeConvexMesh):
            assert np.allclose(
                shape.scale, 1.0
            ), f"Not unit scale {shape.scale}, need to rescale vertices?"
            collision_geom = Convex(vertices=shape.vertices, faces=shape.triangles)
        elif isinstance(shape, PhysxCollisionShapeCylinder):
            collision_geom = Cylinder(radius=shape.radius, lz=shape.half_length * 2)
            # NOTE: physx Cylinder has x-axis along cylinder height
            # FCL Cylinder has z-axis along cylinder height
            pose = pose * Pose(q=euler2quat(0, np.pi / 2, 0))  # type: ignore
        elif isinstance(shape, PhysxCollisionShapePlane):
            raise NotImplementedError(
                "Support for Plane collision is not implemented yet in mplib, "
                "need fcl::Plane"
            )
        elif isinstance(shape, PhysxCollisionShapeSphere):
            collision_geom = Sphere(radius=shape.radius)
        elif isinstance(shape, PhysxCollisionShapeTriangleMesh):
            collision_geom = BVHModel()
            collision_geom.begin_model()
            collision_geom.add_sub_model(vertices=shape.vertices, faces=shape.triangles)
            collision_geom.end_model()
        else:
            raise TypeError(f"Unknown shape type: {type(shape)}")
        return [CollisionObject(collision_geom, pose.p, pose.q)]  # type: ignore


class SapienPlanner(Planner):
    def __init__(
        self,
        sapien_planning_world: SapienPlanningWorld,
        move_group: str,
        *,
        joint_vel_limits: Optional[Sequence[float] | np.ndarray] = None,
        joint_acc_limits: Optional[Sequence[float] | np.ndarray] = None,
    ):
        """
        Creates an mplib.planner.Planner from a SapienPlanningWorld.

        :param sapien_planning_world: a SapienPlanningWorld created from sapien.Scene
        :param move_group: name of the move group (end effector link)
        :param joint_vel_limits: joint velocity limits for time parameterization
        :param joint_acc_limits: joint acceleration limits for time parameterization
        """
        self.planning_world = sapien_planning_world
        self.acm = self.planning_world.get_allowed_collision_matrix()

        if len(planned_arts := self.planning_world.get_planned_articulations()) != 1:
            raise NotImplementedError("Must have exactly one planned articulation")
        self.robot = planned_arts[0]
        self.pinocchio_model = self.robot.get_pinocchio_model()
        self.user_link_names = self.pinocchio_model.get_link_names()
        self.user_joint_names = self.pinocchio_model.get_joint_names()

        self.joint_name_2_idx = {}
        for i, joint in enumerate(self.user_joint_names):
            self.joint_name_2_idx[joint] = i
        self.link_name_2_idx = {}
        for i, link in enumerate(self.user_link_names):
            self.link_name_2_idx[link] = i

        assert (
            move_group in self.user_link_names
        ), f"end-effector not found as one of the links in {self.user_link_names}"
        self.move_group = move_group
        self.robot.set_move_group(self.move_group)
        self.move_group_joint_indices = self.robot.get_move_group_joint_indices()

        self.joint_types = self.pinocchio_model.get_joint_types()
        self.joint_limits = np.concatenate(self.pinocchio_model.get_joint_limits())
        if joint_vel_limits is None:
            joint_vel_limits = np.ones(len(self.move_group_joint_indices))
        if joint_acc_limits is None:
            joint_acc_limits = np.ones(len(self.move_group_joint_indices))
        self.joint_vel_limits = joint_vel_limits
        self.joint_acc_limits = joint_acc_limits
        self.move_group_link_id = self.link_name_2_idx[self.move_group]

        assert (
            len(self.joint_vel_limits)
            == len(self.joint_acc_limits)
            == len(self.move_group_joint_indices)
            <= len(self.joint_limits)
        ), (
            "length of joint_vel_limits, joint_acc_limits, and move_group_joint_indices"
            " should equal and be <= number of total joints. "
            f"{len(self.joint_vel_limits)} == {len(self.joint_acc_limits)} "
            f"== {len(self.move_group_joint_indices)} <= {len(self.joint_limits)}"
        )

        # Mask for joints that have equivalent values (revolute joints with range > 2pi)
        self.equiv_joint_mask = [
            t.startswith("JointModelR") for t in self.joint_types
        ] & (self.joint_limits[:, 1] - self.joint_limits[:, 0] > 2 * np.pi)

        self.planner = ompl.OMPLPlanner(world=self.planning_world)

    def update_from_simulation(self, *, update_attached_object: bool = True) -> None:
        """
        Updates planning_world articulations/objects pose with current Scene state.
        Directly calls ``SapienPlanningWorld.update_from_simulation()``

        :param update_attached_object: whether to update the attached pose of
            all attached objects
        """
        self.planning_world.update_from_simulation(
            update_attached_object=update_attached_object
        )
