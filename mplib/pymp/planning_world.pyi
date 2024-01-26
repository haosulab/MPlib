import typing

import numpy

import mplib.pymp.articulation
import mplib.pymp.fcl

__all__ = ["PlanningWorld", "WorldCollisionResult"]
M = typing.TypeVar("M", bound=int)

class PlanningWorld:
    """
    Planning world for collision checking
    """
    def __init__(
        self,
        articulations: list[mplib.pymp.articulation.ArticulatedModel],
        articulation_names: list[str],
        normal_objects: list[mplib.pymp.fcl.CollisionObject],
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
    def add_articulation(
        self, model: mplib.pymp.articulation.ArticulatedModel, name: str
    ) -> None:
        """
        Add an articulated model to the planning world.

        :param model: articulated model to be added
        :param name: name of the articulated model
        """
    def add_articulations(
        self, models: list[mplib.pymp.articulation.ArticulatedModel], names: list[str]
    ) -> None:
        """
        Add a list of articulated models to the planning world.

        :param models: list of articulated models to be added
        :param names: list of names of the articulated models
        """
    def collide(self, request: mplib.pymp.fcl.CollisionRequest = ...) -> bool:
        """
        Check collision in the planning world.

        :param request: collision request params. Can leave empty for default value
        :return: ``True`` if collision exists
        """
    def collide_full(
        self, index: int = 0, request: mplib.pymp.fcl.CollisionRequest = ...
    ) -> list[...]:
        """
        Check collision between the articulated model and all objects.

        :param index: index of the articulated model
        :param request: collision request params. Can leave empty for default value
        :return: List of WorldCollisionResult objects
        """
    def collide_with_others(
        self, index: int = 0, request: mplib.pymp.fcl.CollisionRequest = ...
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
    def get_articulations(self) -> list[mplib.pymp.articulation.ArticulatedModel]:
        """
        Get the list of articulated models.

        :return: list of articulated models
        """
    def get_normal_object_names(self) -> list[str]:
        """
        Get the names of non-articulated collision objects.

        :return: list of names of non-articulated collision objects
        """
    def get_normal_objects(self) -> list[mplib.pymp.fcl.CollisionObject]:
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
        self, index: int = 0, request: mplib.pymp.fcl.CollisionRequest = ...
    ) -> list[...]:
        """
        Check collision between the articulated model and itself.

        :param index: index of the articulated model
        :param request: collision request params. Can leave empty for default value
        :return: List of WorldCollisionResult objects
        """
    def set_normal_object(
        self, collision_object: str, name: mplib.pymp.fcl.CollisionObject
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
        p_geom: mplib.pymp.fcl.CollisionGeometry,
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
    def res(self) -> mplib.pymp.fcl.CollisionResult:
        """
        the fcl CollisionResult
        """
