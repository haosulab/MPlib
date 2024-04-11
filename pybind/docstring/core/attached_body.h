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

static const char *__doc_mplib_AttachedBodyTpl =
R"doc(Object defining bodies that can be attached to robot links. This is useful when
handling objects picked up by the robot.

Mimicking MoveIt2's ``moveit::core::AttachedBody``

https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1AttachedBody.html)doc";

static const char *__doc_mplib_AttachedBodyTpl_AttachedBodyTpl =
R"doc(
Construct an attached body for a specified link.

:param name: name of the attached body
:param object: collision object of the attached body
:param attached_articulation: robot articulated model to attach to
:param attached_link_id: id of the articulation link to attach to
:param pose: attached pose (relative pose from attached link to object)
:param touch_links: the link names that the attached body touches)doc";

static const char *__doc_mplib_AttachedBodyTpl_getAttachedArticulation =
R"doc(
Gets the articulation that this body is attached to)doc";

static const char *__doc_mplib_AttachedBodyTpl_getAttachedLinkGlobalPose =
R"doc(
Gets the global pose of the articulation link that this body is attached to)doc";

static const char *__doc_mplib_AttachedBodyTpl_getAttachedLinkId =
R"doc(
Gets the articulation link id that this body is attached to)doc";

static const char *__doc_mplib_AttachedBodyTpl_getGlobalPose =
R"doc(
Gets the global pose of the attached object)doc";

static const char *__doc_mplib_AttachedBodyTpl_getName =
R"doc(
Gets the attached object name)doc";

static const char *__doc_mplib_AttachedBodyTpl_getObject =
R"doc(
Gets the attached object (``FCLObjectPtr``))doc";

static const char *__doc_mplib_AttachedBodyTpl_getPose =
R"doc(
Gets the attached pose (relative pose from attached link to object))doc";

static const char *__doc_mplib_AttachedBodyTpl_getTouchLinks =
R"doc(
Gets the link names that the attached body touches)doc";

static const char *__doc_mplib_AttachedBodyTpl_setPose =
R"doc(
Sets the attached pose (relative pose from attached link to object))doc";

static const char *__doc_mplib_AttachedBodyTpl_setTouchLinks =
R"doc(
Sets the link names that the attached body touches)doc";

static const char *__doc_mplib_AttachedBodyTpl_updatePose =
R"doc(
Updates the global pose of the attached object using current state)doc";

/* ----- Begin of custom docstring section ----- */

static const char *__doc_mplib_AttachedBodyTpl_pose =
R"doc(The attached pose (relative pose from attached link to object))doc";

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
