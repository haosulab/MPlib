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

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl = R"doc(Result of the collision checking.)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_WorldCollisionResultTpl =
R"doc(
Default constructor)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_WorldCollisionResultTpl_2 =
R"doc(
Constructor with all members)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_collision_type = R"doc(type of the collision)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_link_name1 = R"doc(link name of the first object in collision)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_link_name2 = R"doc(link name of the second object in collision)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_object_name1 = R"doc(name of the first object)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_object_name2 = R"doc(name of the second object)doc";

static const char *__doc_mplib_collision_detection_WorldCollisionResultTpl_res = R"doc(the fcl CollisionResult)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl = R"doc(Result of minimum distance-to-collision query.)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_WorldDistanceResultTpl =
R"doc(
Default constructor)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_WorldDistanceResultTpl_2 =
R"doc(
Constructor with all members)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_distance_type = R"doc(type of the distance result)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_link_name1 = R"doc(link name of the first object)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_link_name2 = R"doc(link name of the second object)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_min_distance = R"doc(minimum distance between the two objects)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_object_name1 = R"doc(name of the first object)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_object_name2 = R"doc(name of the second object)doc";

static const char *__doc_mplib_collision_detection_WorldDistanceResultTpl_res = R"doc(the fcl DistanceResult)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
