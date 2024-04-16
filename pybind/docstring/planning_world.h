#pragma once

/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by mkdoc.py.
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1, 0))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

static const char *__doc_mplib_PlanningWorldTpl =
R"doc(Planning world for collision checking

Mimicking MoveIt2's ``planning_scene::PlanningScene``,
``collision_detection::World``, ``moveit::core::RobotState``,
``collision_detection::CollisionEnv``

https://moveit.picknik.ai/main/api/html/classplanning__scene_1_1PlanningScene.html
https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1World.html
https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1RobotState.html
https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1CollisionEnv.html)doc";

static const char *__doc_mplib_PlanningWorldTpl_PlanningWorldTpl =
R"doc(
Constructs a PlanningWorld with given (planned) articulations and objects

:param articulations: list of planned articulated models
:param objects: list of non-articulated collision objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_addArticulation =
R"doc(
Adds an articulation (ArticulatedModelPtr) to world

:param model: articulated model to be added
:param planned: whether the articulation is being planned)doc";

static const char *__doc_mplib_PlanningWorldTpl_addObject =
R"doc(
Adds an non-articulated object containing multiple collision objects
(``FCLObjectPtr``) to world

:param fcl_obj: FCLObject to be added)doc";

static const char *__doc_mplib_PlanningWorldTpl_addObject_2 =
R"doc(
Adds an non-articulated object (``CollisionObjectPtr``) with given name to world

:param name: name of the collision object
:param collision_object: collision object to be added)doc";

static const char *__doc_mplib_PlanningWorldTpl_addPointCloud =
R"doc(
Adds a point cloud as a collision object with given name to world

:param name: name of the point cloud collision object
:param vertices: point cloud vertices matrix
:param resolution: resolution of the point in ``octomap::OcTree``)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachBox =
R"doc(
Attaches given box to specified link of articulation (auto touch_links)

:param size: box side length
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to
:param pose: attached pose (relative pose from attached link to object))doc";

static const char *__doc_mplib_PlanningWorldTpl_attachMesh =
R"doc(
Attaches given mesh to specified link of articulation (auto touch_links)

:param mesh_path: path to a mesh file
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to
:param pose: attached pose (relative pose from attached link to object)
:param convex: whether to load mesh as a convex mesh. Default: ``False``.)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachObject =
R"doc(
Attaches existing non-articulated object to specified link of articulation at
its current pose. If the object is currently attached, disallow collision
between the object and previous touch_links.

Updates acm_ to allow collisions between attached object and touch_links.

:param name: name of the non-articulated object to attach
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to
:param touch_links: link names that the attached object touches
:raises ValueError: if non-articulated object with given name does not exist or
    if planned articulation with given name does not exist)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachObject_2 =
R"doc(
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
    if planned articulation with given name does not exist)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachObject_3 =
R"doc(
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
    if planned articulation with given name does not exist)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachObject_4 =
R"doc(
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
    if planned articulation with given name does not exist)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachObject_5 =
R"doc(
Attaches given object (w/ p_geom) to specified link of articulation at given
pose. This is done by removing the object and then adding and attaching object.
As a result, all previous acm_ entries with the object are removed

:param name: name of the non-articulated object to attach
:param p_geom: pointer to a CollisionGeometry object
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to
:param pose: attached pose (relative pose from attached link to object)
:param touch_links: link names that the attached object touches)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachObject_6 =
R"doc(
Attaches given object (w/ p_geom) to specified link of articulation at given
pose. This is done by removing the object and then adding and attaching object.
As a result, all previous acm_ entries with the object are removed.
Automatically sets touch_links as the name of self links that collide with the
object in the current state (auto touch_links).

:param name: name of the non-articulated object to attach
:param p_geom: pointer to a CollisionGeometry object
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to
:param pose: attached pose (relative pose from attached link to object))doc";

static const char *__doc_mplib_PlanningWorldTpl_attachSphere =
R"doc(
Attaches given sphere to specified link of articulation (auto touch_links)

:param radius: sphere radius
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to
:param pose: attached pose (relative pose from attached link to object))doc";

static const char *__doc_mplib_PlanningWorldTpl_checkCollision =
R"doc(
Check full collision (calls ``checkSelfCollision()`` and
``checkRobotCollision()``)

:param request: collision request params.
:return: List of ``WorldCollisionResult`` objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_checkRobotCollision =
R"doc(
Check collision with other scene bodies in the world (planned articulations with
attached objects collide against unplanned articulations and scene objects)

:param request: collision request params.
:return: List of ``WorldCollisionResult`` objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_checkSelfCollision =
R"doc(
Check for self collision (including planned articulation self-collision, planned
articulation-attach collision, attach-attach collision)

:param request: collision request params.
:return: List of ``WorldCollisionResult`` objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_detachObject =
R"doc(
Detaches object with given name. Updates acm_ to disallow collision between the
object and touch_links.

:param name: name of the non-articulated object to detach
:param also_remove: whether to also remove object from world
:return: ``True`` if success, ``False`` if the object with given name is not
    attached)doc";

static const char *__doc_mplib_PlanningWorldTpl_distance =
R"doc(
Compute the minimum distance-to-all-collision (calls ``distanceSelf()`` and
``distanceRobot()``)

:param request: distance request params.
:return: a ``WorldDistanceResult`` object)doc";

static const char *__doc_mplib_PlanningWorldTpl_distanceRobot =
R"doc(
Compute the minimum distance-to-collision between a robot and the world

:param request: distance request params.
:return: a ``WorldDistanceResult`` object)doc";

static const char *__doc_mplib_PlanningWorldTpl_distanceSelf =
R"doc(
Get the minimum distance to self-collision given the robot in current state

:param request: distance request params.
:return: a ``WorldDistanceResult`` object)doc";

static const char *__doc_mplib_PlanningWorldTpl_distanceToCollision =
R"doc(
Compute the minimum distance-to-all-collision. Calls ``distance()``.

Note that this is different from MoveIt2's
``planning_scene::PlanningScene::distanceToCollision()`` where self-collisions
are ignored.

:return: minimum distance-to-all-collision)doc";

static const char *__doc_mplib_PlanningWorldTpl_distanceToRobotCollision =
R"doc(
The distance between the robot model at current state to the nearest collision
(ignoring self-collisions). Calls ``distanceRobot()``.

:return: minimum distance-to-robot-collision)doc";

static const char *__doc_mplib_PlanningWorldTpl_distanceToSelfCollision =
R"doc(
The minimum distance to self-collision given the robot in current state. Calls
``distanceSelf()``.

:return: minimum distance-to-self-collision)doc";

static const char *__doc_mplib_PlanningWorldTpl_getAllowedCollisionMatrix =
R"doc(
Get the current allowed collision matrix)doc";

static const char *__doc_mplib_PlanningWorldTpl_getArticulation =
R"doc(
Gets the articulation (ArticulatedModelPtr) with given name

:param name: name of the articulated model
:return: the articulated model with given name or ``None`` if not found.)doc";

static const char *__doc_mplib_PlanningWorldTpl_getArticulationNames =
R"doc(
Gets names of all articulations in world (unordered))doc";

static const char *__doc_mplib_PlanningWorldTpl_getAttachedObject =
R"doc(
Gets the attached body (AttachedBodyPtr) with given name

:param name: name of the attached body
:return: the attached body with given name or ``None`` if not found.)doc";

static const char *__doc_mplib_PlanningWorldTpl_getObject =
R"doc(
Gets the non-articulated object (``FCLObjectPtr``) with given name

:param name: name of the non-articulated object
:return: the object with given name or ``None`` if not found.)doc";

static const char *__doc_mplib_PlanningWorldTpl_getObjectNames =
R"doc(
Gets names of all objects in world (unordered))doc";

static const char *__doc_mplib_PlanningWorldTpl_getPlannedArticulations =
R"doc(
Gets all planned articulations (ArticulatedModelPtr))doc";

static const char *__doc_mplib_PlanningWorldTpl_hasArticulation =
R"doc(
Check whether the articulation with given name exists

:param name: name of the articulated model
:return: ``True`` if exists, ``False`` otherwise.)doc";

static const char *__doc_mplib_PlanningWorldTpl_hasObject =
R"doc(
Check whether the non-articulated object with given name exists

:param name: name of the non-articulated object
:return: ``True`` if exists, ``False`` otherwise.)doc";

static const char *__doc_mplib_PlanningWorldTpl_isArticulationPlanned =
R"doc(
Check whether the articulation with given name is being planned

:param name: name of the articulated model
:return: ``True`` if exists, ``False`` otherwise.)doc";

static const char *__doc_mplib_PlanningWorldTpl_isObjectAttached =
R"doc(
Check whether the non-articulated object with given name is attached

:param name: name of the non-articulated object
:return: ``True`` if it is attached, ``False`` otherwise.)doc";

static const char *__doc_mplib_PlanningWorldTpl_isStateColliding =
R"doc(
Check if the current state is in collision (with the environment or self
collision).

:return: ``True`` if collision exists)doc";

static const char *__doc_mplib_PlanningWorldTpl_printAttachedBodyPose =
R"doc(
Prints global pose of all attached bodies)doc";

static const char *__doc_mplib_PlanningWorldTpl_removeArticulation =
R"doc(
Removes the articulation with given name if exists. Updates acm_

:param name: name of the articulated model
:return: ``True`` if success, ``False`` if articulation with given name does not
    exist)doc";

static const char *__doc_mplib_PlanningWorldTpl_removeObject =
R"doc(
Removes (and detaches) the collision object with given name if exists. Updates
acm_

:param name: name of the non-articulated collision object
:return: ``True`` if success, ``False`` if the non-articulated object with given
    name does not exist)doc";

static const char *__doc_mplib_PlanningWorldTpl_setArticulationPlanned =
R"doc(
Sets articulation with given name as being planned

:param name: name of the articulated model
:param planned: whether the articulation is being planned
:raises ValueError: if the articulation with given name does not exist)doc";

static const char *__doc_mplib_PlanningWorldTpl_setQpos =
R"doc(
Set qpos of articulation with given name

:param name: name of the articulated model
:param qpos: joint angles of the *movegroup only* // FIXME: double check)doc";

static const char *__doc_mplib_PlanningWorldTpl_setQposAll =
R"doc(
Set qpos of all planned articulations)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
