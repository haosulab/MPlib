import mplib.pymp.planning_world
import typing
import mplib.pymp.articulation
import mplib.pymp.fcl
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "PlanningWorld",
    "WorldCollisionResult"
]


class PlanningWorld():
    def __init__(self, articulations: list[mplib.pymp.articulation.ArticulatedModel], articulation_names: list[str], normal_objects: list[mplib.pymp.fcl.CollisionObject], normal_object_names: list[str], plan_articulation_id: int = 0) -> None: 
        """
            Planning world for collision checking.
            Args:
                articulations: list of articulated models
                articulation_names: list of names for articulated models
                normal_objects: list of non-articulated collision objects
                normal_object_names: list of names for normal collision objects
                plan_articulation_id: index of the articulated model to be used for planning
            Returns:
                PlanningWorld object
        """
    def add_articulation(self, model: mplib.pymp.articulation.ArticulatedModel, name: str) -> None: 
        """
            Add an articulated model to the planning world.
            Args:
                model: articulated model to be added
                name: name of the articulated model
            Returns:
                None
        """
    def add_articulations(self, models: list[mplib.pymp.articulation.ArticulatedModel], names: list[str]) -> None: 
        """
            Add a list of articulated models to the planning world.
            Args:
                models: list of articulated models to be added
                names: list of names of the articulated models
            Returns:
                None
        """
    def collide(self) -> bool: 
        """
            Check collision between all objects.
            Returns:
                True if collision happens
        """
    @staticmethod
    def collide_full(*args, **kwargs) -> typing.Any: 
        """
            Check collision between the articulated model and all objects.
            Args:
                index: index of the articulated model
                request: collision request params. can leave empty for default value
            Returns:
                List of WorldCollisionResult objects
        """
    @staticmethod
    def collide_with_others(*args, **kwargs) -> typing.Any: 
        """
            Check collision between the articulated model and other objects.
            Args:
                index: index of the articulated model
                request: collision request params. can leave empty for default value
            Returns:
                List of WorldCollisionResult objects
        """
    def get_articulations(self) -> list[mplib.pymp.articulation.ArticulatedModel]: 
        """
            Get the list of articulated models.
            Returns:
                list of articulated models as pointers
        """
    def get_normal_objects(self) -> list[mplib.pymp.fcl.CollisionObject]: 
        """
            Get the list of non-articulated collision objects.
            Returns:
                list of non-articulated collision objects
        """
    def print_attached_tool_pose(self) -> None: 
        """
            Print the pose of the attached tool.
            Returns:
                None
        """
    def remove_attach(self) -> None: 
        """
            Remove the attached tool.
            Returns:
                None
        """
    def remove_normal_object(self, name: str) -> bool: 
        """
            Remove a non-articulated collision object from the planning world.
            Args:
                name: name of the non-articulated collision object
            Returns:
                None
        """
    @staticmethod
    def self_collide(*args, **kwargs) -> typing.Any: 
        """
            Check collision between the articulated model and itself.
            Args:
                index: index of the articulated model
                request: collision request params. can leave empty for default value
            Returns:
                List of WorldCollisionResult objects
        """
    def set_normal_object(self, collision_object: str, name: mplib.pymp.fcl.CollisionObject) -> None: 
        """
            Add a non-articulated collision object to the planning world.
            Args:
                name: name of the non-articulated collision object
                collision_object: non-articulated collision object to be added
            Returns:
                None
        """
    def set_qpos(self, index: int, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: 
        """
            Set the joint qpos of the articulated model.
            Args:
                index: index of the articulated model
                qpos: joint angles of the *movegroup only*
            Returns:
                None
        """
    def set_qpos_all(self, qpos: numpy.ndarray[numpy.float64, _Shape[m, 1]]) -> None: 
        """
            Set the joint qpos of all articulated models.
            Args:
                qpos: joint angles of all the models (*movegroup only*)
            Returns:
                None
        """
    def set_use_attach(self, use: bool) -> None: 
        """
            Set whether to use attached tool for collision checking.
            Args:
                use: whether to use attached tool
            Returns:
                None
        """
    def set_use_point_cloud(self, use: bool) -> None: 
        """
            Set whether to use point cloud for collision checking.
            Args:
                use: whether to use point cloud
            Returns:
                None
        """
    def update_attached_box(self, size: numpy.ndarray[numpy.float64, _Shape[3, 1]], link_id: int, pose: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    def update_attached_mesh(self, mesh_path: str, link_id: int, pose: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: 
        """
            Add mesh as the attached tool.
            Args:
                mesh_path: path to the mesh file
                link_id: link id of the attached mesh
                pose: pose of the attached mesh [x, y, z, qw, qx, qy, qz]
            Returns:
                None
        """
    def update_attached_sphere(self, radius: float, link_id: int, pose: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: 
        """
            Add sphere as the attached tool.
            Args:
                radius: radius of the sphere
                link_id: link id of the attached sphere
                pose: pose of the attached sphere [x, y, z, qw, qx, qy, qz]
            Returns:
                None
        """
    def update_attached_tool(self, p_geom: mplib.pymp.fcl.CollisionGeometry, link_id: int, pose: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: 
        """
            Update the attached tool.
            Args:
                p_geom: fcl collision geometry of the attached tool
                link_id: link id of the attached tool
                pose: pose of the attached tool [x, y, z, qw, qx, qy, qz]
            Returns:
                None
        """
    def update_point_cloud(self, vertices: numpy.ndarray[numpy.float64, _Shape[m, 3]], radius: float) -> None: 
        """
            Update the point cloud for collision checking.
            Args:
                vertices: vertices of the point cloud
                radius: radius of each point in the point cloud
            Returns:
                None
        """
    @property
    def use_attach(self) -> bool:
        """
        :type: bool
        """
    @property
    def use_point_cloud(self) -> bool:
        """
        :type: bool
        """
    pass
class WorldCollisionResult():
    """
        Result of the collision checking.
        Attributes:
            res: whether collision happens
            object_name1: name of the first object
            object_name2: name of the second object
            collision_type: type of the collision
            link_name1: link name of the first object in collision
            link_name2: link name of the second object in collision
    """
    @property
    def collision_type(self) -> str:
        """
        :type: str
        """
    @property
    def link_name1(self) -> str:
        """
        :type: str
        """
    @property
    def link_name2(self) -> str:
        """
        :type: str
        """
    @property
    def object_name1(self) -> str:
        """
        :type: str
        """
    @property
    def object_name2(self) -> str:
        """
        :type: str
        """
    @property
    def res(self) -> mplib.pymp.fcl.CollisionResult:
        """
        :type: mplib.pymp.fcl.CollisionResult
        """
    pass
