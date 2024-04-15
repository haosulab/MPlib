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

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl =
R"doc(FCL collision model of an articulation

See https://github.com/flexible-collision-library/fcl)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_FCLModelTpl =
R"doc(
Construct an FCL model from URDF and SRDF files.

:param urdf_filename: path to URDF file, can be relative to the current working
    directory
:param convex: use convex decomposition for collision objects. Default:
    ``False``.
:param verbose: print debug information. Default: ``False``.)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_FCLModelTpl_2 =
R"doc(
Construct an FCL model from URDF and SRDF files.

:param urdf_model: a urdf tree as urdf::ModelInterfaceSharedPtr
:param package_dir: path to replace package_dir for mesh files
:param convex: use convex decomposition for collision objects. Default:
    ``False``.
:param verbose: print debug information. Default: ``False``.)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_checkCollisionWith =
R"doc(
Check for collision in the current state with another ``FCLModel``, ignoring the
distances between links that are allowed to always collide (as specified by
acm).

:param other: another ``FCLModel`` to check collision with
:param acm: allowed collision matrix.
:param request: collision request
:return: List of ``WorldCollisionResult`` objects. If empty, no collision.)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_checkCollisionWith_2 =
R"doc(
Check for collision in the current state with an ``FCLObject``, ignoring the
distances between objects that are allowed to always collide (as specified by
acm).

:param object: an ``FCLObject`` to check collision with
:param acm: allowed collision matrix.
:param request: collision request
:return: List of ``WorldCollisionResult`` objects. If empty, no collision.)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_checkSelfCollision =
R"doc(
Check for self-collision in the current state and returns all found collisions,
ignoring the distances between links that are allowed to always collide (as
specified by acm).

:param request: collision request
:param acm: allowed collision matrix.
:return: List of ``WorldCollisionResult`` objects. If empty, no self-collision.)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_createFromURDFString =
R"doc(
Constructs a FCLModel from URDF string and collision links

:param urdf_string: URDF string (without visual/collision elements for links)
:param collision_links: Vector of collision links as FCLObjectPtr. Format is:
    ``[FCLObjectPtr, ...]``. The collision objects are at the shape's
    local_pose.
:param verbose: print debug information. Default: ``False``.
:return: a unique_ptr to FCLModel)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_distanceSelf =
R"doc(
Get the minimum distance to self-collision given the robot in current state,
ignoring the distances between links that are allowed to always collide (as
specified by acm).

:param request: distance request.
:param acm: allowed collision matrix.
:return: a ``WorldDistanceResult`` object)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_distanceToCollisionWith =
R"doc(
The minimum distance to collision with another ``FCLModel`` given the robot in
current state, ignoring the distances between links that are allowed to always
collide (as specified by acm).

:param other: another ``FCLModel`` to get minimum distance-to-collision with
:param acm: allowed collision matrix.
:return: minimum distance-to-collision with the other ``FCLModel``)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_distanceToCollisionWith_2 =
R"doc(
The minimum distance to collision with an ``FCLObject`` given the robot in
current state, ignoring the distances between objects that are allowed to always
collide (as specified by acm).

:param object: an ``FCLObject`` to get minimum distance-to-collision with
:param acm: allowed collision matrix.
:return: minimum distance-to-collision with the ``FCLObject``)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_distanceToSelfCollision =
R"doc(
The minimum distance to self-collision given the robot in current state,
ignoring the distances between links that are allowed to always collide (as
specified by acm). Calls ``distanceSelf()``.

:param acm: allowed collision matrix.
:return: minimum distance-to-self-collision)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_distanceWith =
R"doc(
Get the minimum distance to collision with another ``FCLModel`` given the robot
in current state, ignoring the distances between links that are allowed to
always collide (as specified by acm).

:param other: another ``FCLModel`` to get minimum distance-to-collision with
:param request: distance request.
:param acm: allowed collision matrix.
:return: a ``WorldDistanceResult`` object)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_distanceWith_2 =
R"doc(
Get the minimum distance to collision with an ``FCLObject`` given the robot in
current state, ignoring the distances between objects that are allowed to always
collide (as specified by acm).

:param object: an ``FCLObject`` to get minimum distance-to-collision with
:param request: distance request.
:param acm: allowed collision matrix.
:return: a ``WorldDistanceResult`` object)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_getCollisionLinkNames =
R"doc(
)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_getCollisionLinkUserIndices =
R"doc(
)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_getCollisionObjects =
R"doc(
Get the collision objects of the FCL model.

:return: all collision objects of the FCL model)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_getCollisionPairs =
R"doc(
Get the collision pairs of the FCL model.

:return: collision pairs of the FCL model. If the FCL model has N collision
    objects, the collision pairs is a list of N*(N-1)/2 pairs minus the disabled
    collision pairs)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_getName =
R"doc(
Get name of the articulated model.

:return: name of the articulated model)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_getUserLinkNames =
R"doc(
)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_isStateColliding =
R"doc(
Check if the current state is in self-collision, ignoring the distances between
links that are allowed to always collide (as specified by acm).

:param acm: allowed collision matrix.
:return: ``True`` if any collision pair collides and ``False`` otherwise.)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_printCollisionPairs =
R"doc(
Print all collision pairs)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_removeCollisionPairsFromSRDF =
R"doc(
Remove collision pairs from SRDF file.

:param srdf_filename: path to SRDF file, can be relative to the current working
    directory)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_removeCollisionPairsFromSRDFString =
R"doc(
Remove collision pairs from SRDF string.

:param srdf_string: SRDF string (only disable_collisions element))doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_setLinkOrder =
R"doc(
Set the link order of the FCL model.

:param names: list of link names in the order that you want to set.)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_setName =
R"doc(
Set name of the articulated model.

:param name: name of the articulated model)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_updateCollisionObjects =
R"doc(
Update the collision objects of the FCL model.

:param link_poses: list of link poses in the order of the link order)doc";

/* ----- Begin of custom docstring section ----- */

static const char *__doc_mplib_collision_detection_fcl_FCLModelTpl_name =
R"doc(Name of the fcl model)doc";

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
