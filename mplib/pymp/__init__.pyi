"""
Motion planning python binding
"""

import typing

import numpy

from . import collision_detection, kinematics, planning

__all__ = [
    "ArticulatedModel",
    "AttachedBody",
    "PlanningWorld",
    "Pose",
    "collision_detection",
    "kinematics",
    "planning",
    "set_global_seed",
]

class ArticulatedModel:
    """
    Supports initialization from URDF and SRDF files, and provides access to
    underlying Pinocchio and FCL models.
    """
    @staticmethod
    def create_from_urdf_string(
        urdf_string: str,
        srdf_string: str,
        collision_links: list[collision_detection.fcl.FCLObject],
        *,
        name: str = None,
        gravity: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ] = ...,
        link_names: list[str] = [],
        joint_names: list[str] = [],
        verbose: bool = False,
    ) -> ArticulatedModel:
        """
        Constructs an ArticulatedModel from URDF/SRDF strings and collision links

        :param urdf_string: URDF string (without visual/collision elements for links)
        :param srdf_string: SRDF string (only disable_collisions element)
        :param collision_links: Vector of collision links as FCLObjectPtr. Format is:
            ``[FCLObjectPtr, ...]``. The collision objects are at the shape's
            local_pose.
        :param name: name of the articulated model to override URDF robot name attribute
        :param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
        :param link_names: list of links that are considered for planning
        :param joint_names: list of joints that are considered for planning
        :param verbose: print debug information. Default: ``False``.
        :return: a unique_ptr to ArticulatedModel
        """
    def __init__(
        self,
        urdf_filename: str,
        srdf_filename: str,
        *,
        name: str = None,
        gravity: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ] = ...,
        link_names: list[str] = [],
        joint_names: list[str] = [],
        convex: bool = False,
        verbose: bool = False,
    ) -> None:
        """
        Construct an articulated model from URDF and SRDF files.

        :param urdf_filename: path to URDF file, can be relative to the current working
            directory
        :param srdf_filename: path to SRDF file, we use it to disable self-collisions
        :param name: name of the articulated model to override URDF robot name attribute
        :param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
        :param link_names: list of links that are considered for planning
        :param joint_names: list of joints that are considered for planning
        :param convex: use convex decomposition for collision objects. Default:
            ``False``.
        :param verbose: print debug information. Default: ``False``.
        """
    def get_base_pose(self) -> Pose:
        """
        Get the base (root) pose of the robot.

        :return: base pose of the robot
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
    def get_name(self) -> str:
        """
        Get name of the articulated model.

        :return: name of the articulated model
        """
    def get_pinocchio_model(self) -> kinematics.pinocchio.PinocchioModel:
        """
        Get the underlying Pinocchio model.

        :return: Pinocchio model used for kinematics and dynamics computations
        """
    def get_pose(self) -> Pose:
        """
        Get the base (root) pose of the robot.

        :return: base pose of the robot
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
    def set_base_pose(self, pose: Pose) -> None:
        """
        Set the base pose of the robot and update all collision links in the
        ``FCLModel``.

        :param pose: base pose of the robot
        """
    @typing.overload
    def set_move_group(self, end_effector: str) -> None:
        """
        Set the move group, i.e. the chain ending in end effector for which to compute
        the forward kinematics for all subsequent queries.

        :param end_effector: name of the end effector link
        """
    @typing.overload
    def set_move_group(self, end_effectors: list[str]) -> None:
        """
        Set the move group but we have multiple end effectors in a chain. I.e., Base -->
        EE1 --> EE2 --> ... --> EEn

        :param end_effectors: list of links extending to the end effector
        """
    def set_pose(self, pose: Pose) -> None:
        """
        Set the base pose of the robot and update all collision links in the
        ``FCLModel``.

        :param pose: base pose of the robot
        """
    def set_qpos(
        self,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
        full: bool = False,
    ) -> None:
        """
        Set the current joint positions and update all collision links in the
        ``FCLModel``.

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
    @property
    def base_pose(self) -> Pose:
        """
        The base (root) pose of the robot
        """
    @base_pose.setter
    def base_pose(self, arg1: Pose) -> None: ...
    @property
    def name(self) -> str:
        """
        Name of the articulated model
        """
    @property
    def pose(self) -> Pose:
        """
        The base (root) pose of the robot
        """
    @pose.setter
    def pose(self, arg1: Pose) -> None: ...
    @property
    def qpos(
        self,
    ) -> numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]]:
        """
        Current qpos of all active joints
        """

class AttachedBody:
    """
    Object defining bodies that can be attached to robot links. This is useful when
    handling objects picked up by the robot.

    Mimicking MoveIt2's ``moveit::core::AttachedBody``

    https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1AttachedBody.html
    """
    def __init__(
        self,
        name: str,
        object: collision_detection.fcl.FCLObject,
        attached_articulation: ArticulatedModel,
        attached_link_id: int,
        pose: Pose,
        touch_links: list[str] = [],
    ) -> None:
        """
        Construct an attached body for a specified link.

        :param name: name of the attached body
        :param object: collision object of the attached body
        :param attached_articulation: robot articulated model to attach to
        :param attached_link_id: id of the articulation link to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :param touch_links: the link names that the attached body touches
        """
    def get_attached_articulation(self) -> ArticulatedModel:
        """
        Gets the articulation that this body is attached to
        """
    def get_attached_link_global_pose(self) -> Pose:
        """
        Gets the global pose of the articulation link that this body is attached to
        """
    def get_attached_link_id(self) -> int:
        """
        Gets the articulation link id that this body is attached to
        """
    def get_global_pose(self) -> Pose:
        """
        Gets the global pose of the attached object
        """
    def get_name(self) -> str:
        """
        Gets the attached object name
        """
    def get_object(self) -> collision_detection.fcl.FCLObject:
        """
        Gets the attached object (``FCLObjectPtr``)
        """
    def get_pose(self) -> Pose:
        """
        Gets the attached pose (relative pose from attached link to object)
        """
    def get_touch_links(self) -> list[str]:
        """
        Gets the link names that the attached body touches
        """
    def set_pose(self, pose: Pose) -> None:
        """
        Sets the attached pose (relative pose from attached link to object)
        """
    def set_touch_links(self, touch_links: list[str]) -> None:
        """
        Sets the link names that the attached body touches
        """
    def update_pose(self) -> None:
        """
        Updates the global pose of the attached object using current state
        """
    @property
    def pose(self) -> Pose:
        """
        The attached pose (relative pose from attached link to object)
        """
    @pose.setter
    def pose(self, arg1: Pose) -> None: ...

class PlanningWorld:
    """
    Planning world for collision checking

    Mimicking MoveIt2's ``planning_scene::PlanningScene``,
    ``collision_detection::World``, ``moveit::core::RobotState``,
    ``collision_detection::CollisionEnv``

    https://moveit.picknik.ai/main/api/html/classplanning__scene_1_1PlanningScene.html
    https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1World.html
    https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1RobotState.html
    https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1CollisionEnv.html
    """
    def __init__(
        self,
        articulations: list[ArticulatedModel],
        objects: list[collision_detection.fcl.FCLObject] = [],
    ) -> None:
        """
        Constructs a PlanningWorld with given (planned) articulations and objects

        :param articulations: list of planned articulated models
        :param objects: list of non-articulated collision objects
        """
    def add_articulation(self, model: ArticulatedModel, planned: bool = False) -> None:
        """
        Adds an articulation (ArticulatedModelPtr) to world

        :param model: articulated model to be added
        :param planned: whether the articulation is being planned
        """
    @typing.overload
    def add_object(self, fcl_obj: collision_detection.fcl.FCLObject) -> None:
        """
        Adds an non-articulated object containing multiple collision objects
        (``FCLObjectPtr``) to world

        :param fcl_obj: FCLObject to be added
        """
    @typing.overload
    def add_object(
        self, name: str, collision_object: collision_detection.fcl.CollisionObject
    ) -> None:
        """
        Adds an non-articulated object (``CollisionObjectPtr``) with given name to world

        :param name: name of the collision object
        :param collision_object: collision object to be added
        """
    def add_point_cloud(
        self,
        name: str,
        vertices: numpy.ndarray[
            tuple[M, typing.Literal[3]], numpy.dtype[numpy.float64]
        ],
        resolution: float = 0.01,
    ) -> None:
        """
        Adds a point cloud as a collision object with given name to world

        :param name: name of the point cloud collision object
        :param vertices: point cloud vertices matrix
        :param resolution: resolution of the point in ``octomap::OcTree``
        """
    def attach_box(
        self,
        size: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
        art_name: str,
        link_id: int,
        pose: Pose,
    ) -> None:
        """
        Attaches given box to specified link of articulation (auto touch_links)

        :param size: box side length
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        """
    def attach_mesh(
        self,
        mesh_path: str,
        art_name: str,
        link_id: int,
        pose: Pose,
        *,
        convex: bool = False,
    ) -> None:
        """
        Attaches given mesh to specified link of articulation (auto touch_links)

        :param mesh_path: path to a mesh file
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :param convex: whether to load mesh as a convex mesh. Default: ``False``.
        """
    @typing.overload
    def attach_object(
        self, name: str, art_name: str, link_id: int, touch_links: list[str]
    ) -> None:
        """
        Attaches existing non-articulated object to specified link of articulation at
        its current pose. If the object is currently attached, disallow collision
        between the object and previous touch_links.

        Updates acm_ to allow collisions between attached object and touch_links.

        :param name: name of the non-articulated object to attach
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param touch_links: link names that the attached object touches
        :raises ValueError: if non-articulated object with given name does not exist or
            if planned articulation with given name does not exist
        """
    @typing.overload
    def attach_object(self, name: str, art_name: str, link_id: int) -> None:
        """
        Attaches existing non-articulated object to specified link of articulation at
        its current pose. If the object is not currently attached, automatically sets
        touch_links as the name of self links that collide with the object in the
        current state.

        Updates acm_ to allow collisions between attached object and touch_links.

        If the object is already attached, the touch_links of the attached object is
        preserved and acm_ remains unchanged.

        :param name: name of the non-articulated object to attach
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :raises ValueError: if non-articulated object with given name does not exist or
            if planned articulation with given name does not exist
        """
    @typing.overload
    def attach_object(
        self, name: str, art_name: str, link_id: int, pose: Pose, touch_links: list[str]
    ) -> None:
        """
        Attaches existing non-articulated object to specified link of articulation at
        given pose. If the object is currently attached, disallow collision between the
        object and previous touch_links.

        Updates acm_ to allow collisions between attached object and touch_links.

        :param name: name of the non-articulated object to attach
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :param touch_links: link names that the attached object touches
        :raises ValueError: if non-articulated object with given name does not exist or
            if planned articulation with given name does not exist
        """
    @typing.overload
    def attach_object(self, name: str, art_name: str, link_id: int, pose: Pose) -> None:
        """
        Attaches existing non-articulated object to specified link of articulation at
        given pose. If the object is not currently attached, automatically sets
        touch_links as the name of self links that collide with the object in the
        current state.

        Updates acm_ to allow collisions between attached object and touch_links.

        If the object is already attached, the touch_links of the attached object is
        preserved and acm_ remains unchanged.

        :param name: name of the non-articulated object to attach
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :raises ValueError: if non-articulated object with given name does not exist or
            if planned articulation with given name does not exist
        """
    @typing.overload
    def attach_object(
        self,
        name: str,
        p_geom: collision_detection.fcl.CollisionGeometry,
        art_name: str,
        link_id: int,
        pose: Pose,
        touch_links: list[str],
    ) -> None:
        """
        Attaches given object (w/ p_geom) to specified link of articulation at given
        pose. This is done by removing the object and then adding and attaching object.
        As a result, all previous acm_ entries with the object are removed

        :param name: name of the non-articulated object to attach
        :param p_geom: pointer to a CollisionGeometry object
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        :param touch_links: link names that the attached object touches
        """
    @typing.overload
    def attach_object(
        self,
        name: str,
        p_geom: collision_detection.fcl.CollisionGeometry,
        art_name: str,
        link_id: int,
        pose: Pose,
    ) -> None:
        """
        Attaches given object (w/ p_geom) to specified link of articulation at given
        pose. This is done by removing the object and then adding and attaching object.
        As a result, all previous acm_ entries with the object are removed.
        Automatically sets touch_links as the name of self links that collide with the
        object in the current state (auto touch_links).

        :param name: name of the non-articulated object to attach
        :param p_geom: pointer to a CollisionGeometry object
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        """
    def attach_sphere(
        self, radius: float, art_name: str, link_id: int, pose: Pose
    ) -> None:
        """
        Attaches given sphere to specified link of articulation (auto touch_links)

        :param radius: sphere radius
        :param art_name: name of the planned articulation to attach to
        :param link_id: index of the link of the planned articulation to attach to
        :param pose: attached pose (relative pose from attached link to object)
        """
    def check_collision(
        self, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[collision_detection.WorldCollisionResult]:
        """
        Check full collision (calls ``checkSelfCollision()`` and
        ``checkRobotCollision()``)

        :param request: collision request params.
        :return: List of ``WorldCollisionResult`` objects
        """
    def check_robot_collision(
        self, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[collision_detection.WorldCollisionResult]:
        """
        Check collision with other scene bodies in the world (planned articulations with
        attached objects collide against unplanned articulations and scene objects)

        :param request: collision request params.
        :return: List of ``WorldCollisionResult`` objects
        """
    def check_self_collision(
        self, request: collision_detection.fcl.CollisionRequest = ...
    ) -> list[collision_detection.WorldCollisionResult]:
        """
        Check for self collision (including planned articulation self-collision, planned
        articulation-attach collision, attach-attach collision)

        :param request: collision request params.
        :return: List of ``WorldCollisionResult`` objects
        """
    def detach_object(self, name: str, also_remove: bool = False) -> bool:
        """
        Detaches object with given name. Updates acm_ to disallow collision between the
        object and touch_links.

        :param name: name of the non-articulated object to detach
        :param also_remove: whether to also remove object from world
        :return: ``True`` if success, ``False`` if the object with given name is not
            attached
        """
    def distance(
        self, request: collision_detection.fcl.DistanceRequest = ...
    ) -> collision_detection.WorldDistanceResult:
        """
        Compute the minimum distance-to-all-collision (calls ``distanceSelf()`` and
        ``distanceRobot()``)

        :param request: distance request params.
        :return: a ``WorldDistanceResult`` object
        """
    def distance_robot(
        self, request: collision_detection.fcl.DistanceRequest = ...
    ) -> collision_detection.WorldDistanceResult:
        """
        Compute the minimum distance-to-collision between a robot and the world

        :param request: distance request params.
        :return: a ``WorldDistanceResult`` object
        """
    def distance_self(
        self, request: collision_detection.fcl.DistanceRequest = ...
    ) -> collision_detection.WorldDistanceResult:
        """
        Get the minimum distance to self-collision given the robot in current state

        :param request: distance request params.
        :return: a ``WorldDistanceResult`` object
        """
    def distance_to_collision(self) -> float:
        """
        Compute the minimum distance-to-all-collision. Calls ``distance()``.

        Note that this is different from MoveIt2's
        ``planning_scene::PlanningScene::distanceToCollision()`` where self-collisions
        are ignored.

        :return: minimum distance-to-all-collision
        """
    def distance_to_robot_collision(self) -> float:
        """
        The distance between the robot model at current state to the nearest collision
        (ignoring self-collisions). Calls ``distanceRobot()``.

        :return: minimum distance-to-robot-collision
        """
    def distance_to_self_collision(self) -> float:
        """
        The minimum distance to self-collision given the robot in current state. Calls
        ``distanceSelf()``.

        :return: minimum distance-to-self-collision
        """
    def get_allowed_collision_matrix(
        self,
    ) -> collision_detection.AllowedCollisionMatrix:
        """
        Get the current allowed collision matrix
        """
    def get_articulation(self, name: str) -> ArticulatedModel:
        """
        Gets the articulation (ArticulatedModelPtr) with given name

        :param name: name of the articulated model
        :return: the articulated model with given name or ``None`` if not found.
        """
    def get_articulation_names(self) -> list[str]:
        """
        Gets names of all articulations in world (unordered)
        """
    def get_attached_object(self, name: str) -> AttachedBody:
        """
        Gets the attached body (AttachedBodyPtr) with given name

        :param name: name of the attached body
        :return: the attached body with given name or ``None`` if not found.
        """
    def get_object(self, name: str) -> collision_detection.fcl.FCLObject:
        """
        Gets the non-articulated object (``FCLObjectPtr``) with given name

        :param name: name of the non-articulated object
        :return: the object with given name or ``None`` if not found.
        """
    def get_object_names(self) -> list[str]:
        """
        Gets names of all objects in world (unordered)
        """
    def get_planned_articulations(self) -> list[ArticulatedModel]:
        """
        Gets all planned articulations (ArticulatedModelPtr)
        """
    def has_articulation(self, name: str) -> bool:
        """
        Check whether the articulation with given name exists

        :param name: name of the articulated model
        :return: ``True`` if exists, ``False`` otherwise.
        """
    def has_object(self, name: str) -> bool:
        """
        Check whether the non-articulated object with given name exists

        :param name: name of the non-articulated object
        :return: ``True`` if exists, ``False`` otherwise.
        """
    def is_articulation_planned(self, name: str) -> bool:
        """
        Check whether the articulation with given name is being planned

        :param name: name of the articulated model
        :return: ``True`` if exists, ``False`` otherwise.
        """
    def is_object_attached(self, name: str) -> bool:
        """
        Check whether the non-articulated object with given name is attached

        :param name: name of the non-articulated object
        :return: ``True`` if it is attached, ``False`` otherwise.
        """
    def is_state_colliding(self) -> bool:
        """
        Check if the current state is in collision (with the environment or self
        collision).

        :return: ``True`` if collision exists
        """
    def print_attached_body_pose(self) -> None:
        """
        Prints global pose of all attached bodies
        """
    def remove_articulation(self, name: str) -> bool:
        """
        Removes the articulation with given name if exists. Updates acm_

        :param name: name of the articulated model
        :return: ``True`` if success, ``False`` if articulation with given name does not
            exist
        """
    def remove_object(self, name: str) -> bool:
        """
        Removes (and detaches) the collision object with given name if exists. Updates
        acm_

        :param name: name of the non-articulated collision object
        :return: ``True`` if success, ``False`` if the non-articulated object with given
            name does not exist
        """
    def set_articulation_planned(self, name: str, planned: bool) -> None:
        """
        Sets articulation with given name as being planned

        :param name: name of the articulated model
        :param planned: whether the articulation is being planned
        :raises ValueError: if the articulation with given name does not exist
        """
    def set_qpos(
        self,
        name: str,
        qpos: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Set qpos of articulation with given name

        :param name: name of the articulated model
        :param qpos: joint angles of the *movegroup only* // FIXME: double check
        """
    def set_qpos_all(
        self,
        state: numpy.ndarray[tuple[M, typing.Literal[1]], numpy.dtype[numpy.float64]],
    ) -> None:
        """
        Set qpos of all planned articulations
        """

class Pose:
    """
    Pose stored as a unit quaternion and a position vector

    This struct is intended to be used only for interfacing with Python. Internally,
    ``Pose`` is converted to and stored as ``Eigen::Isometry3`` which is used by all
    computations.
    """
    def __getstate__(self) -> tuple: ...
    def __imul__(self, other: Pose) -> Pose:
        """
        Overloading operator *= for ``Pose<S> *= Pose<S>``
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructs a default Pose with p = (0,0,0) and q = (1,0,0,0)
        """
    @typing.overload
    def __init__(
        self,
        p: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ] = ...,
        q: numpy.ndarray[
            tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]
        ] = ...,
    ) -> None:
        """
        Constructs a Pose with given position and quaternion

        :param p: position, format: (x, y, z)
        :param q: quaternion (can be unnormalized), format: (w, x, y, z)
        """
    @typing.overload
    def __init__(
        self,
        matrix: numpy.ndarray[
            tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Constructs a Pose with given transformation matrix
        (4x4 np.ndarray with np.float64 dtype)

        :param matrix: a 4x4 np.float64 np.ndarray transformation matrix
        """
    @typing.overload
    def __init__(self, obj: typing.Any) -> None:
        """
        Constructs a Pose with given Python object that has ``p`` and ``q`` attributes
        (e.g., ``sapien.Pose``) or a 4x4 np.ndarray transformation matrix.

        :param obj: a Pose-like object with ``p`` and ``q`` attributes or
            a 4x4 np.ndarray transformation matrix
        """
    @typing.overload
    def __mul__(
        self,
        v: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Overloading operator * for ``Pose<S> * Vector3<S>``
        """
    @typing.overload
    def __mul__(self, other: Pose) -> Pose:
        """
        Overloading operator * for ``Pose<S> * Pose<S>``
        """
    def __repr__(self) -> str: ...
    def __setstate__(self, arg0: tuple) -> None: ...
    def distance(self, other: Pose) -> float:
        """
        Computes the distance between two poses by ``norm(p1.p - p2.p) + min(norm(p1.q -
        p2.q), norm(p1.q + p2.q))`.

        The quaternion part has range [0, sqrt(2)].

        :param other: the other pose
        :return: the distance between the two poses
        """
    def get_p(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Gets the position part of the Pose

        :return: position, format: (x, y, z)
        """
    def get_q(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Gets the quaternion part of the Pose

        :return: quaternion, format: (w, x, y, z)
        """
    def inv(self) -> Pose:
        """
        Get the inserse Pose

        :return: the inverse Pose
        """
    def set_p(
        self,
        p: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Sets the position part of the Pose

        :param p: position, format: (x, y, z)
        """
    def set_q(
        self,
        q: numpy.ndarray[
            tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None:
        """
        Sets the quaternion part of the Pose

        :param q: quaternion (can be unnormalized), format: (w, x, y, z)
        """
    def to_transformation_matrix(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[4], typing.Literal[4]], numpy.dtype[numpy.float64]
    ]:
        """
        Constructs a transformation matrix from this Pose

        :return: a 4x4 transformation matrix
        """
    @property
    def p(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Position part of the Pose (x, y, z)
        """
    @p.setter
    def p(
        self,
        arg0: numpy.ndarray[
            tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None: ...
    @property
    def q(
        self,
    ) -> numpy.ndarray[
        tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]
    ]:
        """
        Quaternion part of the Pose (w, x, y, z)
        """
    @q.setter
    def q(
        self,
        arg1: numpy.ndarray[
            tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]
        ],
    ) -> None: ...

def set_global_seed(seed: int) -> None:
    """
    Sets the global seed for MPlib (``std::srand()``, OMPL's RNG, and FCL's RNG).

    :param seed: the random seed value
    """
