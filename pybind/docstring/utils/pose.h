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

static const char *__doc_mplib_Pose =
R"doc(Pose stored as a unit quaternion and a position vector

This struct is intended to be used only for interfacing with Python. Internally,
``Pose`` is converted to and stored as ``Eigen::Isometry3`` which is used by all
computations.)doc";

static const char *__doc_mplib_Pose_Pose =
R"doc(
Constructs a default Pose with p = (0,0,0) and q = (1,0,0,0))doc";

static const char *__doc_mplib_Pose_Pose_2 =
R"doc(
Constructs a Pose with given position and quaternion

:param p: position, format: (x, y, z)
:param q: quaternion (can be unnormalized), format: (w, x, y, z))doc";

static const char *__doc_mplib_Pose_Pose_3 =
R"doc(
Constructs a Pose with given position and quaternion

:param p: position, format: (x, y, z)
:param q: quaternion (can be unnormalized), format: (w, x, y, z))doc";

static const char *__doc_mplib_Pose_Pose_4 =
R"doc(
Constructs a Pose from a given ``Eigen::Isometry3`` instance

:param pose: an ``Eigen::Isometry3`` instance)doc";

static const char *__doc_mplib_Pose_distance =
R"doc(
Computes the distance between two poses by ``norm(p1.p - p2.p) + min(norm(p1.q -
p2.q), norm(p1.q + p2.q))`.

The quaternion part has range [0, sqrt(2)].

:param other: the other pose
:return: the distance between the two poses)doc";

static const char *__doc_mplib_Pose_inverse =
R"doc(
Get the inserse Pose

:return: the inverse Pose)doc";

static const char *__doc_mplib_Pose_operator_imul =
R"doc(
Overloading operator *= for ``Pose<S> *= Pose<S>``)doc";

static const char *__doc_mplib_Pose_operator_mul =
R"doc(
Overloading operator * for ``Pose<S> * Vector3<S>``)doc";

static const char *__doc_mplib_Pose_operator_mul_2 =
R"doc(
Overloading operator * for ``Pose<S> * Pose<S>``)doc";

static const char *__doc_mplib_Pose_p = R"doc(Position part of the Pose (x, y, z))doc";

static const char *__doc_mplib_Pose_q = R"doc(Quaternion part of the Pose (w, x, y, z))doc";

static const char *__doc_mplib_Pose_toIsometry =
R"doc(
Converts the Pose to an ``Eigen::Isometry3`` instance

:return: an ``Eigen::Isometry3`` instance)doc";

/* ----- Begin of custom docstring section ----- */

static const char *__doc_mplib_Pose_Pose_5 =
R"doc(
Constructs a Pose with given transformation matrix
(4x4 np.ndarray with np.float64 dtype)

:param matrix: a 4x4 np.float64 np.ndarray transformation matrix)doc";

static const char *__doc_mplib_Pose_Pose_6 =
R"doc(
Constructs a Pose with given Python object that has ``p`` and ``q`` attributes
(e.g., ``sapien.Pose``) or a 4x4 np.ndarray transformation matrix.

:param obj: a Pose-like object with ``p`` and ``q`` attributes or
    a 4x4 np.ndarray transformation matrix)doc";

static const char *__doc_mplib_Pose_to_transformation_matrix =
R"doc(
Constructs a transformation matrix from this Pose

:return: a 4x4 transformation matrix)doc";

static const char *__doc_mplib_Pose_set_p =
R"doc(
Sets the position part of the Pose

:param p: position, format: (x, y, z))doc";

static const char *__doc_mplib_Pose_get_p =
R"doc(
Gets the position part of the Pose

:return: position, format: (x, y, z))doc";

static const char *__doc_mplib_Pose_set_q =
R"doc(
Sets the quaternion part of the Pose

:param q: quaternion (can be unnormalized), format: (w, x, y, z))doc";

static const char *__doc_mplib_Pose_get_q =
R"doc(
Gets the quaternion part of the Pose

:return: quaternion, format: (w, x, y, z))doc";

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
