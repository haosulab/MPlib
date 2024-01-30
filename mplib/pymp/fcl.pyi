import typing

import numpy

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
    "load_mesh_as_Convex",
]
M = typing.TypeVar("M", bound=int)

class BVHModel(CollisionGeometry):
    """
    BVHModel collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a mesh geometry
    represented by a BVH tree.
    """
    def __init__(self) -> None: ...
    @typing.overload
    def addSubModel(
        self,
        vertices: list[
            numpy.ndarray[
                tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
            ]
        ],
    ) -> int:
        """
        Add a sub-model to the BVHModel.

        :param vertices: vertices of the sub-model
        """
    @typing.overload
    def addSubModel(
        self,
        vertices: list[
            numpy.ndarray[
                tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
            ]
        ],
        faces: list[Triangle],
    ) -> int:
        """
        Add a sub-model to the BVHModel.

        :param vertices: vertices of the sub-model
        :param faces: faces of the sub-model represented by a list of vertex indices
        """
    @typing.overload
    def addSubModel(
        self,
        vertices: list[
            numpy.ndarray[
                tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
            ]
        ],
        faces: list[
            numpy.ndarray[
                tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.int32]
            ]
        ],
    ) -> None:
        """
        Add a sub-model to the BVHModel.

        :param vertices: vertices of the sub-model
        :param faces: faces of the sub-model represented by a list of vertex indices
        """
    def beginModel(self, num_faces: int = 0, num_vertices: int = 0) -> int:
        """
        Begin to construct a BVHModel.

        :param num_faces: number of faces of the mesh
        :param num_vertices: number of vertices of the mesh
        """
    def endModel(self) -> int:
        """
        End the construction of a BVHModel.
        """
    def get_faces(self) -> list[Triangle]:
        """
        Get the faces of the BVHModel.

        :return: faces of the BVHModel
        """
    def get_vertices(
        self,
    ) -> list[
        numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ]
    ]:
        """
        Get the vertices of the BVHModel.

        :return: vertices of the BVHModel
        """
    @property
    def num_faces(self) -> int: ...
    @property
    def num_vertices(self) -> int: ...

class Box(CollisionGeometry):
    """
    Box collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a box geometry.
    """

    side: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    @typing.overload
    def __init__(
        self,
        side: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Construct a box with given side length.

        :param side: side length of the box in an array [x, y, z]
        """
    @typing.overload
    def __init__(self, x: float, y: float, z: float) -> None:
        """
        Construct a box with given side length.

        :param x: side length of the box in x direction
        :param y: side length of the box in y direction
        :param z: side length of the box in z direction
        """

class Capsule(CollisionGeometry):
    """
    Capsule collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a capsule geometry.
    """

    lz: float
    radius: float
    def __init__(self, radius: float, lz: float) -> None:
        """
        Construct a capsule with given radius and height.

        :param radius: radius of the capsule
        :param lz: height of the capsule
        """

class CollisionGeometry:
    """
    Collision geometry base class.

    This is an FCL class so you can refer to the FCL doc here.
    https://flexible-collision-library.github.io/d6/d5d/classfcl_1_1CollisionGeometry.html
    """

    aabb_center: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    aabb_radius: float
    cost_density: float
    def computeCOM(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...
    def computeLocalAABB(self) -> None: ...
    def computeMomentofInertia(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]
    ]: ...
    def computeMomentofInertiaRelatedToCOM(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]
    ]: ...
    def computeVolume(self) -> float: ...
    def isFree(self) -> bool: ...
    def isOccupied(self) -> bool: ...
    def isUncertain(self) -> bool: ...

class CollisionObject:
    """
    Collision object class.

    This class contains the collision geometry and the transformation of the
    geometry.
    """
    def __init__(
        self,
        collision_geometry: CollisionGeometry,
        translation: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        rotation: numpy.ndarray[
            tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Construct a collision object with given collision geometry and transformation.

        :param collision_geometry: collision geometry of the object
        :param translation: translation of the object
        :param rotation: rotation of the object
        """
    def get_collision_geometry(self) -> CollisionGeometry: ...
    def get_rotation(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]
    ]: ...
    def get_translation(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...
    def set_transformation(
        self,
        arg0: numpy.ndarray[
            tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None: ...

class CollisionRequest:
    def __init__(
        self,
        num_max_contacts: int = 1,
        enable_contact: bool = False,
        num_max_cost_sources: int = 1,
        enable_cost: bool = False,
        use_approximate_cost: bool = True,
        gjk_solver_type: GJKSolverType = ...,
        gjk_tolerance: float = 1e-06,
    ) -> None: ...
    def isSatisfied(self, result: ...) -> bool: ...

class CollisionResult:
    def __init__(self) -> None: ...
    def add_contact(self, c: ...) -> None: ...
    def add_cost_source(self, c: ..., num_max_cost_sources: int) -> None: ...
    def clear(self) -> None: ...
    def get_contact(self, i: int) -> ...: ...
    def get_contacts(self) -> list[...]: ...
    def get_cost_sources(self) -> list[...]: ...
    def is_collision(self) -> bool: ...
    def num_contacts(self) -> int: ...
    def num_cost_sources(self) -> int: ...

class Contact:
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(
        self, o1: CollisionGeometry, o2: CollisionGeometry, b1: int, b2: int
    ) -> None: ...
    @typing.overload
    def __init__(
        self,
        o1: CollisionGeometry,
        o2: CollisionGeometry,
        b1: int,
        b2: int,
        pos: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        normal: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        depth: float,
    ) -> None: ...
    @property
    def normal(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...
    @property
    def penetration_depth(self) -> float: ...
    @property
    def pos(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...

class ContactPoint:
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(
        self,
        normal: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        pos: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        penetration_depth: float,
    ) -> None: ...
    @property
    def normal(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...
    @property
    def penetration_depth(self) -> float: ...
    @property
    def pos(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...

class Convex(CollisionGeometry):
    """
    Convex collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a convex geometry.
    """
    @staticmethod
    @typing.overload
    def __init__(*args, **kwargs) -> None:
        """
        Construct a convex with given vertices and faces.

        :param vertices: vertices of the convex
        :param num_faces: number of faces of the convex
        :param faces: faces of the convex geometry represented by a list of vertex
            indices
        :param throw_if_invalid: if ``True``, throw an exception if the convex is
            invalid
        """
    @typing.overload
    def __init__(
        self,
        vertices: numpy.ndarray[
            tuple[M, typing.Literal[3]], numpy.dtype[numpy.float64]
        ],
        faces: numpy.ndarray[tuple[M, typing.Literal[3]], numpy.dtype[numpy.int32]],
        throw_if_invalid: bool = True,
    ) -> None:
        """
        Construct a convex with given vertices and faces.

        :param vertices: vertices of the convex
        :param faces: faces of the convex geometry represented by a list of vertex
            indices
        :param throw_if_invalid: if ``True``, throw an exception if the convex is
            invalid
        """
    def compute_volume(self) -> float:
        """
        Compute the volume of the convex.

        :return: volume of the convex
        """
    def get_face_count(self) -> int:
        """
        Get the number of faces of the convex.

        :return: number of faces of the convex
        """
    def get_faces(self) -> list[int]:
        """
        Get the faces of the convex.

        :return: faces of the convex represented by a list of vertex indices
        """
    def get_interior_point(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Sample a random interior point of the convex geometry

        :return: interior point of the convex
        """
    def get_vertices(
        self,
    ) -> list[
        numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ]
    ]:
        """
        Get the vertices of the convex.

        :return: vertices of the convex
        """

class CostSource:
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(
        self,
        aabb_min: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        aabb_max: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        cost_density: float,
    ) -> None: ...
    @property
    def aabb_max(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...
    @property
    def aabb_min(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...
    @property
    def cost_density(self) -> float: ...
    @property
    def total_cost(self) -> float: ...

class Cylinder(CollisionGeometry):
    """
    Cylinder collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a cylinder
    geometry.
    """

    lz: float
    radius: float
    def __init__(self, radius: float, lz: float) -> None:
        """
        Construct a cylinder with given radius and height.

        :param radius: radius of the cylinder
        :param lz: height of the cylinder
        """

class DistanceRequest:
    def __init__(
        self,
        enable_nearest_points: bool = False,
        enable_signed_distance: bool = False,
        rel_err: float = 0.0,
        abs_err: float = 0.0,
        distance_tolerance: float = 1e-06,
        gjk_solver_type: GJKSolverType = ...,
    ) -> None: ...
    def isSatisfied(self, result: ...) -> bool: ...

class DistanceResult:
    def __init__(self, min_distance: float = 1.7976931348623157e308) -> None: ...
    def clear(self) -> None: ...
    @property
    def min_distance(self) -> float: ...
    @property
    def nearest_points(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...

class FCLModel:
    """
    FCL collision model of an articulation

    See https://github.com/flexible-collision-library/fcl
    """
    def __init__(
        self, urdf_filename: str, verbose: bool = True, convex: bool = False
    ) -> None:
        """
        Construct an FCL model from URDF and SRDF files.

        :param urdf_filename: path to URDF file, can be relative to the current working
            directory
        :param verbose: print debug information
        :param convex: use convex decomposition for collision objects
        """
    def collide(self, request: CollisionRequest = ...) -> bool:
        """
        Perform collision checking.

        :param request: collision request
        :return: ``True`` if collision happens
        """
    def collide_full(
        self, request: CollisionRequest = ...
    ) -> list[CollisionResult]: ...
    def get_collision_link_names(self) -> list[str]: ...
    def get_collision_objects(self) -> list[CollisionObject]:
        """
        Get the collision objects of the FCL model.

        :return: all collision objects of the FCL model
        """
    def get_collision_pairs(self) -> list[tuple[int, int]]:
        """
        Get the collision pairs of the FCL model.

        :return: collision pairs of the FCL model. If the FCL model has N collision
            objects, the collision pairs is a list of N*(N-1)/2 pairs minus the disabled
            collision pairs
        """
    def remove_collision_pairs_from_srdf(self, srdf_filename: str) -> None:
        """
        Remove collision pairs from SRDF.

        :param srdf_filename: path to SRDF file, can be relative to the current working
            directory
        """
    def set_link_order(self, names: list[str]) -> None:
        """
        Set the link order of the FCL model.

        :param names: list of link names in the order that you want to set.
        """
    def update_collision_objects(
        self,
        link_poses: list[
            numpy.ndarray[
                tuple[typing.Literal[7], typing.Literal[1]], numpy.dtype[numpy.float64]
            ]
        ],
    ) -> None:
        """
        Update the collision objects of the FCL model.

        :param link_poses: list of link poses in the order of the link order
        """

class GJKSolverType:
    """
    Members:

      GST_LIBCCD

      GST_INDEP
    """

    GST_INDEP: typing.ClassVar[GJKSolverType]  # value = <GJKSolverType.GST_INDEP: 1>
    GST_LIBCCD: typing.ClassVar[GJKSolverType]  # value = <GJKSolverType.GST_LIBCCD: 0>
    __members__: typing.ClassVar[
        dict[str, GJKSolverType]
    ]  # value = {'GST_LIBCCD': <GJKSolverType.GST_LIBCCD: 0>, 'GST_INDEP': <GJKSolverType.GST_INDEP: 1>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class OcTree(CollisionGeometry):
    """
    OcTree collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a point cloud
    geometry represented by an OcTree.
    """
    @typing.overload
    def __init__(self, resolution: float) -> None:
        """
        Construct an OcTree with given resolution.

        :param resolution: resolution of the OcTree (smallest size of a voxel).
            You can treat this is as the diameter of a point.
        """
    @typing.overload
    def __init__(
        self,
        vertices: numpy.ndarray[
            tuple[M, typing.Literal[3]], numpy.dtype[numpy.float64]
        ],
        resolution: float,
    ) -> None:
        """
        Construct an OcTree with given vertices and resolution.

        :param vertices: vertices of the point cloud
        :param resolution: resolution of the OcTree
        """

class Triangle:
    def __getitem__(self, arg0: int) -> int: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, arg0: int, arg1: int, arg2: int) -> None: ...
    def get(self, arg0: int) -> int: ...
    def set(self, arg0: int, arg1: int, arg2: int) -> None: ...

def collide(
    arg0: CollisionObject, arg1: CollisionObject, arg2: CollisionRequest
) -> CollisionResult: ...
def distance(
    arg0: CollisionObject, arg1: CollisionObject, arg2: DistanceRequest
) -> DistanceResult: ...
def load_mesh_as_BVH(
    mesh_path: str,
    scale: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ],
) -> BVHModel: ...
def load_mesh_as_Convex(
    mesh_path: str,
    scale: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ],
) -> Convex: ...

GST_INDEP: GJKSolverType  # value = <GJKSolverType.GST_INDEP: 1>
GST_LIBCCD: GJKSolverType  # value = <GJKSolverType.GST_LIBCCD: 0>