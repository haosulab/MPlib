import mplib.pymp.fcl
import typing
import GJKSolverType
import numpy
_Shape = typing.Tuple[int, ...]

__all__ = [
    "BVHModel",
    "Box",
    "Capsule",
    "CollisionGeometry",
    "CollisionObject",
    "CollisionRequest",
    "CollisionResult",
    "Contact",
    "ContactPoint",
    "Convex",
    "CostSource",
    "Cylinder",
    "DistanceRequest",
    "DistanceResult",
    "FCLModel",
    "GJKSolverType",
    "GST_INDEP",
    "GST_LIBCCD",
    "OcTree",
    "Triangle",
    "collide",
    "distance",
    "load_mesh_as_BVH",
    "load_mesh_as_Convex"
]


class CollisionGeometry():
    def computeCOM(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def computeLocalAABB(self) -> None: ...
    def computeMomentofInertia(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]: ...
    def computeMomentofInertiaRelatedToCOM(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]: ...
    def computeVolume(self) -> float: ...
    def isFree(self) -> bool: ...
    def isOccupied(self) -> bool: ...
    def isUncertain(self) -> bool: ...
    @property
    def aabb_center(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @aabb_center.setter
    def aabb_center(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    @property
    def aabb_radius(self) -> float:
        """
        :type: float
        """
    @aabb_radius.setter
    def aabb_radius(self, arg0: float) -> None:
        pass
    @property
    def cost_density(self) -> float:
        """
        :type: float
        """
    @cost_density.setter
    def cost_density(self, arg0: float) -> None:
        pass
    pass
class Box(CollisionGeometry):
    @typing.overload
    def __init__(self, side: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None: ...
    @typing.overload
    def __init__(self, x: float, y: float, z: float) -> None: ...
    @property
    def side(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @side.setter
    def side(self, arg0: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> None:
        pass
    pass
class Capsule(CollisionGeometry):
    def __init__(self, radius: float, lz: float) -> None: ...
    @property
    def lz(self) -> float:
        """
        :type: float
        """
    @lz.setter
    def lz(self, arg0: float) -> None:
        pass
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    @radius.setter
    def radius(self, arg0: float) -> None:
        pass
    pass
class BVHModel(CollisionGeometry):
    def __init__(self) -> None: ...
    @typing.overload
    def addSubModel(self, vertices: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]) -> int: ...
    @typing.overload
    def addSubModel(self, vertices: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], faces: typing.List[Triangle]) -> int: ...
    @typing.overload
    def addSubModel(self, vertices: typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]], faces: typing.List[numpy.ndarray[numpy.int32, _Shape[3, 1]]]) -> None: ...
    def beginModel(self, num_faces: int = 0, num_vertices: int = 0) -> int: ...
    def endModel(self) -> int: ...
    def get_faces(self) -> typing.List[Triangle]: ...
    def get_vertices(self) -> typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    @property
    def num_faces(self) -> int:
        """
        :type: int
        """
    @property
    def num_vertices(self) -> int:
        """
        :type: int
        """
    pass
class CollisionObject():
    def __init__(self, arg0: CollisionGeometry, arg1: numpy.ndarray[numpy.float64, _Shape[3, 1]], arg2: numpy.ndarray[numpy.float64, _Shape[4, 1]]) -> None: ...
    def get_collision_geometry(self) -> CollisionGeometry: ...
    def get_rotation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 3]]: ...
    def get_translation(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def set_transformation(self, arg0: numpy.ndarray[numpy.float64, _Shape[7, 1]]) -> None: ...
    pass
class CollisionRequest():
    def __init__(self, num_max_contacts: int = 1, enable_contact: bool = False, num_max_cost_sources: int = 1, enable_cost: bool = False, use_approximate_cost: bool = True, gjk_solver_type: GJKSolverType = GJKSolverType.GST_LIBCCD, gjk_tolerance: float = 1e-06) -> None: ...
    @staticmethod
    def isSatisfied(*args, **kwargs) -> typing.Any: ...
    pass
class CollisionResult():
    def __init__(self) -> None: ...
    @staticmethod
    def add_contact(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def add_cost_source(*args, **kwargs) -> typing.Any: ...
    def clear(self) -> None: ...
    @staticmethod
    def get_contact(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def get_contacts(*args, **kwargs) -> typing.Any: ...
    @staticmethod
    def get_cost_sources(*args, **kwargs) -> typing.Any: ...
    def is_collision(self) -> bool: ...
    def num_contacts(self) -> int: ...
    def num_cost_sources(self) -> int: ...
    pass
class Contact():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, o1: CollisionGeometry, o2: CollisionGeometry, b1: int, b2: int) -> None: ...
    @typing.overload
    def __init__(self, o1: CollisionGeometry, o2: CollisionGeometry, b1: int, b2: int, pos: numpy.ndarray[numpy.float64, _Shape[3, 1]], normal: numpy.ndarray[numpy.float64, _Shape[3, 1]], depth: float) -> None: ...
    @property
    def normal(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @property
    def penetration_depth(self) -> float:
        """
        :type: float
        """
    @property
    def pos(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    pass
class ContactPoint():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, normal: numpy.ndarray[numpy.float64, _Shape[3, 1]], pos: numpy.ndarray[numpy.float64, _Shape[3, 1]], penetration_depth: float) -> None: ...
    @property
    def normal(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @property
    def penetration_depth(self) -> float:
        """
        :type: float
        """
    @property
    def pos(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    pass
class Convex(CollisionGeometry):
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> typing.Any: ...
    @typing.overload
    def __init__(self, vertices: numpy.ndarray[numpy.float64, _Shape[m, 3]], faces: numpy.ndarray[numpy.int32, _Shape[m, 3]], throw_if_invalid: bool = True) -> None: ...
    def compute_volume(self) -> float: ...
    def get_face_count(self) -> int: ...
    def get_faces(self) -> typing.List[int]: ...
    def get_interior_point(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]: ...
    def get_vertices(self) -> typing.List[numpy.ndarray[numpy.float64, _Shape[3, 1]]]: ...
    pass
class CostSource():
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, aabb_min: numpy.ndarray[numpy.float64, _Shape[3, 1]], aabb_max: numpy.ndarray[numpy.float64, _Shape[3, 1]], cost_density: float) -> None: ...
    @property
    def aabb_max(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @property
    def aabb_min(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    @property
    def cost_density(self) -> float:
        """
        :type: float
        """
    @property
    def total_cost(self) -> float:
        """
        :type: float
        """
    pass
class Cylinder(CollisionGeometry):
    def __init__(self, radius: float, lz: float) -> None: ...
    @property
    def lz(self) -> float:
        """
        :type: float
        """
    @lz.setter
    def lz(self, arg0: float) -> None:
        pass
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    @radius.setter
    def radius(self, arg0: float) -> None:
        pass
    pass
class DistanceRequest():
    def __init__(self, enable_nearest_points: bool = False, enable_signed_distance: bool = False, rel_err: float = 0.0, abs_err: float = 0.0, distance_tolerance: float = 1e-06, gjk_solver_type: GJKSolverType = GJKSolverType.GST_LIBCCD) -> None: ...
    @staticmethod
    def isSatisfied(*args, **kwargs) -> typing.Any: ...
    pass
class DistanceResult():
    def __init__(self, min_distance: float = 1.7976931348623157e+308) -> None: ...
    def clear(self) -> None: ...
    @property
    def min_distance(self) -> float:
        """
        :type: float
        """
    @property
    def nearest_points(self) -> numpy.ndarray[numpy.float64, _Shape[3, 1]]:
        """
        :type: numpy.ndarray[numpy.float64, _Shape[3, 1]]
        """
    pass
class FCLModel():
    def __init__(self, urdf_filename: str, verbose: bool = True, convex: bool = False) -> None: ...
    def collide(self, request: CollisionRequest = ...) -> bool: ...
    def collide_full(self, request: CollisionRequest = ...) -> typing.List[CollisionResult]: ...
    def get_collision_link_names(self) -> typing.List[str]: ...
    def get_collision_objects(self) -> typing.List[CollisionObject]: ...
    def get_collision_pairs(self) -> typing.List[typing.Tuple[int, int]]: ...
    def remove_collision_pairs_from_srdf(self, srdf_filename: str) -> None: ...
    def set_link_order(self, names: typing.List[str]) -> None: ...
    def update_collision_objects(self, link_poses: typing.List[numpy.ndarray[numpy.float64, _Shape[7, 1]]]) -> None: ...
    pass
class GJKSolverType():
    """
    Members:

      GST_LIBCCD

      GST_INDEP
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    GST_INDEP: mplib.pymp.fcl.GJKSolverType # value = <GJKSolverType.GST_INDEP: 1>
    GST_LIBCCD: mplib.pymp.fcl.GJKSolverType # value = <GJKSolverType.GST_LIBCCD: 0>
    __members__: dict # value = {'GST_LIBCCD': <GJKSolverType.GST_LIBCCD: 0>, 'GST_INDEP': <GJKSolverType.GST_INDEP: 1>}
    pass
class OcTree(CollisionGeometry):
    @typing.overload
    def __init__(self, resolution: float) -> None: ...
    @typing.overload
    def __init__(self, vertices: numpy.ndarray[numpy.float64, _Shape[m, 3]], resolution: float) -> None: ...
    pass
class Triangle():
    def __getitem__(self, arg0: int) -> int: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: int, arg1: int, arg2: int) -> None: ...
    def get(self, arg0: int) -> int: ...
    def set(self, arg0: int, arg1: int, arg2: int) -> None: ...
    pass
def collide(arg0: CollisionObject, arg1: CollisionObject, arg2: CollisionRequest) -> CollisionResult:
    pass
def distance(arg0: CollisionObject, arg1: CollisionObject, arg2: DistanceRequest) -> DistanceResult:
    pass
def load_mesh_as_BVH(mesh_path: str, scale: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> BVHModel:
    pass
def load_mesh_as_Convex(mesh_path: str, scale: numpy.ndarray[numpy.float64, _Shape[3, 1]]) -> Convex:
    pass
GST_INDEP: mplib.pymp.fcl.GJKSolverType # value = <GJKSolverType.GST_INDEP: 1>
GST_LIBCCD: mplib.pymp.fcl.GJKSolverType # value = <GJKSolverType.GST_LIBCCD: 0>
