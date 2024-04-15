"""
FCL submodule
"""

import typing

import numpy

import mplib.pymp
import mplib.pymp.collision_detection

__all__ = [
    "BVHModel",
    "Box",
    "Capsule",
    "CollisionGeometry",
    "CollisionObject",
    "CollisionRequest",
    "CollisionResult",
    "Cone",
    "Contact",
    "ContactPoint",
    "Convex",
    "CostSource",
    "Cylinder",
    "DistanceRequest",
    "DistanceResult",
    "Ellipsoid",
    "FCLModel",
    "FCLObject",
    "GJKSolverType",
    "GST_INDEP",
    "GST_LIBCCD",
    "Halfspace",
    "OcTree",
    "Plane",
    "Sphere",
    "Triangle",
    "TriangleP",
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
    def add_sub_model(
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
    def add_sub_model(
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
    def add_sub_model(
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
    def begin_model(self, num_faces: int = 0, num_vertices: int = 0) -> int:
        """
        Begin to construct a BVHModel.

        :param num_faces: number of faces of the mesh
        :param num_vertices: number of vertices of the mesh
        """
    def end_model(self) -> int:
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
        :param lz: height of the capsule along z axis
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
    def compute_com(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]: ...
    def compute_local_aabb(self) -> None: ...
    def compute_moment_of_inertia(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]
    ]: ...
    def compute_moment_of_inertia_related_to_com(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[3]], numpy.dtype[numpy.float64]
    ]: ...
    def compute_volume(self) -> float: ...
    def is_free(self) -> bool: ...
    def is_occupied(self) -> bool: ...
    def is_uncertain(self) -> bool: ...

class CollisionObject:
    """
    Collision object class.

    This class contains the collision geometry and the transformation of the
    geometry.
    """
    def __init__(
        self, collision_geometry: CollisionGeometry, pose: mplib.pymp.Pose = ...
    ) -> None:
        """
        Construct a collision object with given collision geometry and transformation.

        :param collision_geometry: collision geometry of the object
        :param pose: pose of the object
        """
    def get_collision_geometry(self) -> CollisionGeometry: ...
    def get_pose(self) -> mplib.pymp.Pose:
        """
        Gets the current pose of the collision object in world

        :return: The current pose of the collision object
        """
    def set_pose(self, pose: mplib.pymp.Pose) -> None:
        """
        Sets the pose of the collision object in world

        :param pose: New pose of the collision object
        """
    @property
    def pose(self) -> mplib.pymp.Pose:
        """
        Pose of the collision object in world
        """
    @pose.setter
    def pose(self, arg1: mplib.pymp.Pose) -> None: ...

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
    def is_satisfied(self, result: ...) -> bool: ...

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

class Cone(CollisionGeometry):
    """
    Cone collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a cone geometry.
    """

    lz: float
    radius: float
    def __init__(self, radius: float, lz: float) -> None:
        """
        Construct a cone with given radius and height.

        :param radius: radius of the cone
        :param lz: height of the cone along z axis
        """

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
        :param lz: height of the cylinder along z axis
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
    def is_satisfied(self, result: ...) -> bool: ...

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

class Ellipsoid(CollisionGeometry):
    """
    Ellipsoid collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a ellipsoid
    geometry.
    """

    radii: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    @typing.overload
    def __init__(self, a: float, b: float, c: float) -> None:
        """
        Construct a ellipsoid with given parameters.

        :param a: length of the ``x`` semi-axis
        :param b: length of the ``y`` semi-axis
        :param c: length of the ``z`` semi-axis
        """
    @typing.overload
    def __init__(
        self,
        radii: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Construct a ellipsoid with given parameters.

        :param radii: vector of the length of the ``x``, ``y``, and ``z`` semi-axes
        """

class FCLModel:
    """
    FCL collision model of an articulation

    See https://github.com/flexible-collision-library/fcl
    """
    @staticmethod
    def create_from_urdf_string(
        urdf_string: str, collision_links: list[...], *, verbose: bool = False
    ) -> FCLModel:
        """
        Constructs a FCLModel from URDF string and collision links

        :param urdf_string: URDF string (without visual/collision elements for links)
        :param collision_links: Vector of collision links as FCLObjectPtr. Format is:
            ``[FCLObjectPtr, ...]``. The collision objects are at the shape's
            local_pose.
        :param verbose: print debug information. Default: ``False``.
        :return: a unique_ptr to FCLModel
        """
    def __init__(
        self, urdf_filename: str, *, convex: bool = False, verbose: bool = False
    ) -> None:
        """
        Construct an FCL model from URDF and SRDF files.

        :param urdf_filename: path to URDF file, can be relative to the current working
            directory
        :param convex: use convex decomposition for collision objects. Default:
            ``False``.
        :param verbose: print debug information. Default: ``False``.
        """
    @typing.overload
    def check_collision_with(
        self,
        other: FCLModel,
        request: CollisionRequest = ...,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> list[mplib.pymp.collision_detection.WorldCollisionResult]:
        """
        Check for collision in the current state with another ``FCLModel``, ignoring the
        distances between links that are allowed to always collide (as specified by
        acm).

        :param other: another ``FCLModel`` to check collision with
        :param acm: allowed collision matrix.
        :param request: collision request
        :return: List of ``WorldCollisionResult`` objects. If empty, no collision.
        """
    @typing.overload
    def check_collision_with(
        self,
        object: ...,
        request: CollisionRequest = ...,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> list[mplib.pymp.collision_detection.WorldCollisionResult]:
        """
        Check for collision in the current state with an ``FCLObject``, ignoring the
        distances between objects that are allowed to always collide (as specified by
        acm).

        :param object: an ``FCLObject`` to check collision with
        :param acm: allowed collision matrix.
        :param request: collision request
        :return: List of ``WorldCollisionResult`` objects. If empty, no collision.
        """
    def check_self_collision(
        self,
        request: CollisionRequest = ...,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> list[mplib.pymp.collision_detection.WorldCollisionResult]:
        """
        Check for self-collision in the current state and returns all found collisions,
        ignoring the distances between links that are allowed to always collide (as
        specified by acm).

        :param request: collision request
        :param acm: allowed collision matrix.
        :return: List of ``WorldCollisionResult`` objects. If empty, no self-collision.
        """
    def distance_self(
        self,
        request: DistanceRequest = ...,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> mplib.pymp.collision_detection.WorldDistanceResult:
        """
        Get the minimum distance to self-collision given the robot in current state,
        ignoring the distances between links that are allowed to always collide (as
        specified by acm).

        :param request: distance request.
        :param acm: allowed collision matrix.
        :return: a ``WorldDistanceResult`` object
        """
    @typing.overload
    def distance_to_collision_with(
        self,
        other: FCLModel,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> float:
        """
        The minimum distance to collision with another ``FCLModel`` given the robot in
        current state, ignoring the distances between links that are allowed to always
        collide (as specified by acm).

        :param other: another ``FCLModel`` to get minimum distance-to-collision with
        :param acm: allowed collision matrix.
        :return: minimum distance-to-collision with the other ``FCLModel``
        """
    @typing.overload
    def distance_to_collision_with(
        self,
        object: ...,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> float:
        """
        The minimum distance to collision with an ``FCLObject`` given the robot in
        current state, ignoring the distances between objects that are allowed to always
        collide (as specified by acm).

        :param object: an ``FCLObject`` to get minimum distance-to-collision with
        :param acm: allowed collision matrix.
        :return: minimum distance-to-collision with the ``FCLObject``
        """
    def distance_to_self_collision(
        self, *, acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...
    ) -> float:
        """
        The minimum distance to self-collision given the robot in current state,
        ignoring the distances between links that are allowed to always collide (as
        specified by acm). Calls ``distanceSelf()``.

        :param acm: allowed collision matrix.
        :return: minimum distance-to-self-collision
        """
    @typing.overload
    def distance_with(
        self,
        other: FCLModel,
        request: DistanceRequest = ...,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> mplib.pymp.collision_detection.WorldDistanceResult:
        """
        Get the minimum distance to collision with another ``FCLModel`` given the robot
        in current state, ignoring the distances between links that are allowed to
        always collide (as specified by acm).

        :param other: another ``FCLModel`` to get minimum distance-to-collision with
        :param request: distance request.
        :param acm: allowed collision matrix.
        :return: a ``WorldDistanceResult`` object
        """
    @typing.overload
    def distance_with(
        self,
        object: ...,
        request: DistanceRequest = ...,
        *,
        acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...,
    ) -> mplib.pymp.collision_detection.WorldDistanceResult:
        """
        Get the minimum distance to collision with an ``FCLObject`` given the robot in
        current state, ignoring the distances between objects that are allowed to always
        collide (as specified by acm).

        :param object: an ``FCLObject`` to get minimum distance-to-collision with
        :param request: distance request.
        :param acm: allowed collision matrix.
        :return: a ``WorldDistanceResult`` object
        """
    def get_collision_link_names(self) -> list[str]: ...
    def get_collision_objects(self) -> list[...]:
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
    def get_name(self) -> str:
        """
        Get name of the articulated model.

        :return: name of the articulated model
        """
    def is_state_colliding(
        self, *, acm: mplib.pymp.collision_detection.AllowedCollisionMatrix = ...
    ) -> bool:
        """
        Check if the current state is in self-collision, ignoring the distances between
        links that are allowed to always collide (as specified by acm).

        :param acm: allowed collision matrix.
        :return: ``True`` if any collision pair collides and ``False`` otherwise.
        """
    def remove_collision_pairs_from_srdf(self, srdf_filename: str) -> None:
        """
        Remove collision pairs from SRDF file.

        :param srdf_filename: path to SRDF file, can be relative to the current working
            directory
        """
    def set_link_order(self, names: list[str]) -> None:
        """
        Set the link order of the FCL model.

        :param names: list of link names in the order that you want to set.
        """
    def update_collision_objects(self, link_poses: list[mplib.pymp.Pose]) -> None:
        """
        Update the collision objects of the FCL model.

        :param link_poses: list of link poses in the order of the link order
        """
    @property
    def name(self) -> str:
        """
        Name of the fcl model
        """

class FCLObject:
    """
    A general high-level object which consists of multiple FCLCollisionObjects. It
    is the top level data structure which is used in the collision checking process.

    Mimicking MoveIt2's ``collision_detection::FCLObject`` and
    ``collision_detection::World::Object``

    https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1FCLObject.html
    https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1World_1_1Object.html
    """
    @typing.overload
    def __init__(self, name: str) -> None:
        """
        Construct a new FCLObject with the given name

        :param name: name of this FCLObject
        """
    @typing.overload
    def __init__(
        self,
        name: str,
        pose: mplib.pymp.Pose,
        shapes: list[CollisionObject],
        shape_poses: list[mplib.pymp.Pose],
    ) -> None:
        """
        Construct a new FCLObject with the given name and shapes

        :param name: name of this FCLObject
        :param pose: pose of this FCLObject. All shapes are relative to this pose
        :param shapes: all collision shapes as a vector of ``fcl::CollisionObjectPtr``
        :param shape_poses: relative poses from this FCLObject to each collision shape
        """
    @property
    def name(self) -> str:
        """
        Name of this FCLObject
        """
    @property
    def pose(self) -> mplib.pymp.Pose:
        """
        Pose of this FCLObject. All shapes are relative to this pose
        """
    @property
    def shape_poses(self) -> list[mplib.pymp.Pose]:
        """
        Relative poses from this FCLObject to each collision shape
        """
    @property
    def shapes(self) -> list[CollisionObject]:
        """
        All collision shapes (``fcl::CollisionObjectPtr``) making up this FCLObject
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

class Halfspace(CollisionGeometry):
    """
    Infinite halfspace collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a halfspace geometry.
    """

    d: float
    n: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    @typing.overload
    def __init__(
        self,
        n: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        d: float,
    ) -> None:
        """
        Construct a halfspace with given normal direction and offset where ``n * p = d``.
        Points in the negative side of the separation plane ``{p | n * p < d}`` are inside
        the half space (will have collision).

        :param n: normal direction of the halfspace
        :param d: offset of the halfspace
        """
    @typing.overload
    def __init__(self, a: float, b: float, c: float, d: float) -> None:
        """
        Construct a halfspace with given halfspace parameters where ``ax + by + cz = d``.
        Points in the negative side of the separation plane ``{(x, y, z) | ax + by + cz < d}``
        are inside the half space (will have collision).
        """
    def distance(
        self,
        p: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> float:
        """
        Compute the distance of a point to the halfspace as ``abs(n * p - d)``.

        :param p: a point in 3D space
        :return: distance of the point to the halfspace
        """
    def signed_distance(
        self,
        p: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> float:
        """
        Compute the signed distance of a point to the halfspace as ``n * p - d``.

        :param p: a point in 3D space
        :return: signed distance of the point to the halfspace
        """

class OcTree(CollisionGeometry):
    """
    OcTree collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a point cloud
    geometry represented by an OcTree.
    """
    @typing.overload
    def __init__(self, resolution: float = 0.01) -> None:
        """
        Construct an OcTree with given resolution.

        :param resolution: resolution of the OcTree (smallest size of a voxel).
            You can treat this is as the diameter of a point. Default is 0.01.
        """
    @typing.overload
    def __init__(
        self,
        vertices: numpy.ndarray[
            tuple[M, typing.Literal[3]], numpy.dtype[numpy.float64]
        ],
        resolution: float = 0.01,
    ) -> None:
        """
        Construct an OcTree with given vertices and resolution.

        :param vertices: vertices of the point cloud
        :param resolution: resolution of the OcTree. Default is 0.01
        """

class Plane(CollisionGeometry):
    """
    Infinite plane collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a plane geometry.
    """

    d: float
    n: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    @typing.overload
    def __init__(
        self,
        n: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        d: float,
    ) -> None:
        """
        Construct a plane with given normal direction and offset where ``n * p = d``.

        :param n: normal direction of the plane
        :param d: offset of the plane
        """
    @typing.overload
    def __init__(self, a: float, b: float, c: float, d: float) -> None:
        """
        Construct a plane with given plane parameters where ``ax + by + cz = d``.
        """
    def distance(
        self,
        p: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> float:
        """
        Compute the distance of a point to the plane as ``abs(n * p - d)``.

        :param p: a point in 3D space
        :return: distance of the point to the plane
        """
    def signed_distance(
        self,
        p: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> float:
        """
        Compute the signed distance of a point to the plane as ``n * p - d``.

        :param p: a point in 3D space
        :return: signed distance of the point to the plane
        """

class Sphere(CollisionGeometry):
    """
    Sphere collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a sphere geometry.
    """

    radius: float
    def __init__(self, radius: float) -> None:
        """
        Construct a sphere with given radius.

        :param radius: radius of the sphere
        """

class Triangle:
    """
    Triangle with 3 indices for points.

    This is an FCL class so you can refer to the FCL doc here.
    https://flexible-collision-library.github.io/de/daa/classfcl_1_1Triangle.html
    """
    def __getitem__(self, arg0: int) -> int: ...
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, p1: int, p2: int, p3: int) -> None: ...
    def get(self, arg0: int) -> int: ...
    def set(self, arg0: int, arg1: int, arg2: int) -> None: ...

class TriangleP(CollisionGeometry):
    """
    TriangleP collision geometry.

    Inheriting from CollisionGeometry, this class specializes to a triangleP geometry.
    """

    a: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    b: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    c: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]
    def __init__(
        self,
        a: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        b: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        c: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Construct a set of triangles from vectors of point coordinates.

        :param a: vector of point ``x`` coordinates
        :param b: vector of point ``y`` coordinates
        :param c: vector of point ``z`` coordinates
        """

@typing.overload
def collide(
    obj1: CollisionObject, obj2: CollisionObject, request: CollisionRequest = ...
) -> CollisionResult: ...
@typing.overload
def collide(
    obj1: FCLObject, obj2: FCLObject, request: CollisionRequest = ...
) -> CollisionResult: ...
@typing.overload
def distance(
    obj1: CollisionObject, obj2: CollisionObject, request: DistanceRequest = ...
) -> DistanceResult: ...
@typing.overload
def distance(
    obj1: FCLObject, obj2: FCLObject, request: DistanceRequest = ...
) -> DistanceResult: ...
def load_mesh_as_BVH(
    mesh_path: str,
    scale: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ],
) -> BVHModel:
    """
    Load a triangle mesh from mesh_path as a non-convex collision object.

    :param mesh_path: path to the mesh
    :param scale: mesh scale factor
    :return: a shared_ptr to an fcl::BVHModel_OBBRSS<S> collision object
    """

def load_mesh_as_Convex(
    mesh_path: str,
    scale: numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ],
) -> Convex:
    """
    Load a convex mesh from mesh_path.

    :param mesh_path: path to the mesh
    :param scale: mesh scale factor
    :return: a shared_ptr to an fcl::Convex<S> collision object
    :raises RuntimeError: if the mesh is not convex.
    """

GST_INDEP: GJKSolverType  # value = <GJKSolverType.GST_INDEP: 1>
GST_LIBCCD: GJKSolverType  # value = <GJKSolverType.GST_LIBCCD: 0>
