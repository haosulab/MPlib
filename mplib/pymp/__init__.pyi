"""
Motion planning python binding
"""

import typing

import numpy

from . import collision_detection, kinematics, planning

__all__ = [
    "ArticulatedModel",
    "PlanningWorld",
    "WorldCollisionResult",
    "collision_detection",
    "kinematics",
    "planning",
]

class ArticulatedModel:
    """
    Supports initialization from URDF and SRDF files, and provides access to
    underlying Pinocchio and FCL models.
    """
    def __init__(
        self,
        urdf_filename: str,
        srdf_filename: str,
        gravity: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        link_names: list[str],
        joint_names: list[str],
        convex: bool = False,
        verbose: bool = False,
    ) -> None:
        """
        Construct an articulated model from URDF and SRDF files.

        :param urdf_filename: path to URDF file, can be relative to the current working
            directory
        :param srdf_filename: path to SRDF file, we use it to disable self-collisions
        :param gravity: gravity vector
        :param link_names: list of links that are considered for planning
        :param joint_names: list of joints that are considered for planning
        :param convex: use convex decomposition for collision objects. Default:
            ``False``.
        :param verbose: print debug information. Default: ``False``.
        """
    def get_base_pose(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Get the base pose of the robot.

        :return: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    def get_fcl_model(self) -> collision_detection.fcl.FCLModel:
        """
        Get the underlying FCL model.

        :return: FCL model used for collision checking
        """
    def get_move_group_end_effectors(self) -> list[str]:
        """
        Get the end effectors of the move group.

        :return: list of end effectors of the move group
        """
    def get_move_group_joint_indices(self) -> list[int]:
        """
        Get the joint indices of the move group.

        :return: list of user joint indices of the move group
        """
    def get_move_group_joint_names(self) -> list[str]:
        """
        Get the joint names of the move group.

        :return: list of joint names of the move group
        """
    def get_move_group_qpos_dim(self) -> int:
        """
        Get the dimension of the move group qpos.

        :return: dimension of the move group qpos
        """
    def get_pinocchio_model(self) -> kinematics.pinocchio.PinocchioModel:
        """
        Get the underlying Pinocchio model.

        :return: Pinocchio model used for kinematics and dynamics computations
        """
    def get_qpos(
        self,
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Get the current joint position of all active joints inside the URDF.

        :return: current qpos of all active joints
        """
    def get_user_joint_names(self) -> list[str]:
        """
        Get the joint names that the user has provided for planning.

        :return: list of joint names of the user
        """
    def get_user_link_names(self) -> list[str]:
        """
        Get the link names that the user has provided for planning.

        :return: list of link names of the user
        """
    def set_base_pose(
        self,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Set the base pose of the robot.

        :param pose: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
        """
    @typing.overload
    def set_move_group(self, end_effector: str) -> None:
        """
        Set the move group, i.e. the chain ending in end effector for which to compute
        the forward kinematics for all subsequent queries.

        :param chain: list of links extending to the end effector
        """
    @typing.overload
    def set_move_group(self, end_effectors: list[str]) -> None:
        """
        Set the move group but we have multiple end effectors in a chain. I.e., Base -->
        EE1 --> EE2 --> ... --> EEn

        :param end_effectors: names of the end effector link
        """
    def set_qpos(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        full: bool = False,
    ) -> None:
        """
        Let the planner know the current joint positions.

        :param qpos: current qpos of all active joints or just the move group joints
        :param full: whether to set the full qpos or just the move group qpos. If full
            is ``False``, we will pad the missing joints with current known qpos. The
            default is ``False``
        """
    def update_SRDF(self, SRDF: str) -> None:
        """
        Update the SRDF file to disable self-collisions.

        :param srdf: path to SRDF file, can be relative to the current working directory
        """

class PlanningWorld:
    """
    Planning world for collision checking
    """
    def __init__(
        self,
        articulations: list[ArticulatedModel],
        articulation_names: list[str],
        normal_objects: list[collision_detection.fcl.CollisionObject],
        normal_object_names: list[str],
        plan_articulation_id: int = 0,
    ) -> None:
        """
        Constructs a PlanningWorld with given articulations and normal objects

        :param articulations: list of articulated models
        :param articulation_names: name of the articulated models
        :param normal_objects: list of collision objects that are not articulated
        :param normal_object_names: name of the normal objects
        :param plan_articulation_id: id of the articulated model that is used for
            planning
        """
    def add_articulation(self, model: ArticulatedModel, name: str) -> None:
        """
        Add an articulated model to the planning world.

        :param model: articulated model to be added
        :param name: name of the articulated model
        """
    def add_articulations(
        self, models: list[ArticulatedModel], names: list[str]
    ) -> None:
        """
        Add a list of articulated models to the planning world.

        :param models: list of articulated models to be added
        :param names: list of names of the articulated models
        """
    def collide(self, request: collision_detection.fcl.CollisionRequest = ...) -> bool:
        """
        Check collision in the planning world.

        :param request: collision request params. Can leave empty for default value
        :return: ``True`` if collision exists
        """
    def collide_full(
        self, index: int = 0, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[...]:
        """
        Check collision between the articulated model and all objects.

        :param index: index of the articulated model
        :param request: collision request params. Can leave empty for default value
        :return: List of WorldCollisionResult objects
        """
    def collide_with_others(
        self, index: int = 0, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[...]:
        """
        Check collision between the articulated model and other objects.

        :param index: index of the articulated model
        :param request: collision request params. Can leave empty for default value
        :return: List of WorldCollisionResult objects
        """
    def get_articulation_names(self) -> list[str]:
        """
        Get the names of articulated models.

        :return: list of names of articulated models
        """
    def get_articulations(self) -> list[ArticulatedModel]:
        """
        Get the list of articulated models.

        :return: list of articulated models
        """
    def get_normal_object_names(self) -> list[str]:
        """
        Get the names of non-articulated collision objects.

        :return: list of names of non-articulated collision objects
        """
    def get_normal_objects(self) -> list[collision_detection.fcl.CollisionObject]:
        """
        Get the list of non-articulated collision objects.

        :return: list of non-articulated collision objects
        """
    def print_attached_tool_pose(self) -> None:
        """
        Print the pose of the attached tool.
        """
    def remove_attach(self) -> None:
        """
        Remove attach object so there won't be anything on the end effector when
        ``use_attach`` is set to ``True`` again
        """
    def remove_normal_object(self, name: str) -> bool:
        """
        Remove am non-articulated object

        :param name: name of the non-articulated collision object
        :return: ``True`` if the item exists and ``False`` otherwise
        """
    def self_collide(
        self, index: int = 0, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[...]:
        """
        Check collision between the articulated model and itself.

        :param index: index of the articulated model
        :param request: collision request params. Can leave empty for default value
        :return: List of WorldCollisionResult objects
        """
    def set_normal_object(
        self, collision_object: str, name: collision_detection.fcl.CollisionObject
    ) -> None:
        """
        Add a non-articulated collision object to the planning world.

        :param name: name of the non-articulated collision object
        :param collision_object: the non-articulated collision object to be added
        """
    def set_qpos(
        self,
        index: int,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Set the joint qpos of the articulated model.

        :param index: index of the articulated model
        :param qpos: joint angles of the *movegroup only*
        """
    def set_qpos_all(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Set the joint qpos of all articulated models.

        :param qpos: joint angles of all the models (*movegroup only*)
        """
    def set_use_attach(self, use: bool) -> None:
        """
        Set whether to use attached tool for collision checking.

        :param use: whether to use attached tool
        """
    def set_use_point_cloud(self, use: bool) -> None:
        """
        Set whether to use point cloud for collision checking.

        :param use: whether to use point cloud
        """
    def update_attached_box(
        self,
        size: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Add a box as the attached tool.

        :param size: size of the box, [size_x, size_y, size_z]
        :param link_id: link id of the attached box
        :param pose: pose of the attached box w.r.t. the link it's attached to. [x, y,
            z, qw, qx, qy, qz]
        """
    def update_attached_mesh(
        self,
        mesh_path: str,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Add a mesh as the attached tool.

        :param mesh_path: path to the mesh file
        :param link_id: link id of the attached mesh
        :param pose: pose of the attached mesh w.r.t. the link it's attached to. [x, y,
            z, qw, qx, qy, qz]
        """
    def update_attached_sphere(
        self,
        radius: float,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Add a sphere as the attached tool.

        :param radius: radius of the sphere
        :param link_id: link id of the attached sphere
        :param pose: pose of the attached sphere w.r.t. the link it's attached to. [x,
            y, z, qw, qx, qy, qz]
        """
    def update_attached_tool(
        self,
        p_geom: collision_detection.fcl.CollisionGeometry,
        link_id: int,
        pose: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Attach or update the attached object

        :param p_geom: fcl collision geometry of the attached tool
        :param link_id: id of the link to which the object is attached
        :param pose: pose of the attached object w.r.t. the link it's attached to. [x,
            y, z, qw, qx, qy, qz]
        """
    def update_point_cloud(
        self,
        vertices: numpy.ndarray[
            tuple[M, typing.Literal[3]], numpy.dtype[numpy.float64]
        ],
        radius: float,
    ) -> None:
        """
        Update the point cloud for collision checking.

        :param vertices: vertices of the point cloud
        :param radius: radius of each point in the point cloud
        """
    @property
    def use_attach(self) -> bool: ...
    @property
    def use_point_cloud(self) -> bool: ...

class WorldCollisionResult:
    """
    Result of the collision checking.
    """
    @property
    def collision_type(self) -> str:
        """
        type of the collision
        """
    @property
    def link_name1(self) -> str:
        """
        link name of the first object in collision
        """
    @property
    def link_name2(self) -> str:
        """
        link name of the second object in collision
        """
    @property
    def object_name1(self) -> str:
        """
        name of the first object
        """
    @property
    def object_name2(self) -> str:
        """
        name of the second object
        """
    @property
    def res(self) -> collision_detection.fcl.CollisionResult:
        """
        the fcl CollisionResult
        """
