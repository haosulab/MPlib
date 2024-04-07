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

static const char *__doc_mplib_collision_detection_fcl_FCLObject =
R"doc(A general high-level object which consists of multiple FCLCollisionObjects. It
is the top level data structure which is used in the collision checking process.

Mimicking MoveIt2's ``collision_detection::FCLObject`` and
``collision_detection::World::Object``

https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1FCLObject.html
https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1World_1_1Object.html)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLObject_FCLObject =
R"doc(
Construct a new FCLObject with the given name

:param name: name of this FCLObject)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLObject_FCLObject_2 =
R"doc(
Construct a new FCLObject with the given name and shapes

:param name: name of this FCLObject
:param pose: pose of this FCLObject. All shapes are relative to this pose
:param shapes: all collision shapes as a vector of ``fcl::CollisionObjectPtr``
:param shape_poses: relative poses from this FCLObject to each collision shape)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLObject_name = R"doc(Name of this FCLObject)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLObject_pose = R"doc(Pose of this FCLObject. All shapes are relative to this pose)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLObject_shape_poses = R"doc(Relative poses from this FCLObject to each collision shape)doc";

static const char *__doc_mplib_collision_detection_fcl_FCLObject_shapes = R"doc(All collision shapes (``fcl::CollisionObjectPtr``) making up this FCLObject)doc";

static const char *__doc_mplib_collision_detection_fcl_collide =
R"doc(
Collision function between two ``FCLObject``

:param obj1: the first object
:param obj2: the second object
:param request: ``fcl::CollisionRequest``
:param result: ``fcl::CollisionResult``
:return: number of contacts generated between the two objects)doc";

static const char *__doc_mplib_collision_detection_fcl_distance =
R"doc(
Distance function between two ``FCLObject``

:param obj1: the first object
:param obj2: the second object
:param request: ``fcl::DistanceRequest``
:param result: ``fcl::DistanceResult``
:return: minimum distance generated between the two objects)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
