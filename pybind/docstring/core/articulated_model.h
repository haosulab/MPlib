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

static const char *__doc_mplib_ArticulatedModelTpl =
R"doc(Supports initialization from URDF and SRDF files, and provides access to
underlying Pinocchio and FCL models.)doc";

static const char *__doc_mplib_ArticulatedModelTpl_ArticulatedModelTpl =
R"doc(
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
:param verbose: print debug information. Default: ``False``.)doc";

static const char *__doc_mplib_ArticulatedModelTpl_ArticulatedModelTpl_2 =
R"doc(
Dummy default constructor that is protected by Secret. Used by
``createFromURDFString()`` only)doc";

static const char *__doc_mplib_ArticulatedModelTpl_createFromURDFString =
R"doc(
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
:return: a unique_ptr to ArticulatedModel)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getBasePose =
R"doc(
Get the base (root) pose of the robot.

:return: base pose of the robot)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getFCLModel =
R"doc(
Get the underlying FCL model.

:return: FCL model used for collision checking)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getMoveGroupEndEffectors =
R"doc(
Get the end effectors of the move group.

:return: list of end effectors of the move group)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getMoveGroupJointIndices =
R"doc(
Get the joint indices of the move group.

:return: list of user joint indices of the move group)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getMoveGroupJointNames =
R"doc(
Get the joint names of the move group.

:return: list of joint names of the move group)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getName =
R"doc(
Get name of the articulated model.

:return: name of the articulated model)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getPinocchioModel =
R"doc(
Get the underlying Pinocchio model.

:return: Pinocchio model used for kinematics and dynamics computations)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getQpos =
R"doc(
Get the current joint position of all active joints inside the URDF.

:return: current qpos of all active joints)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getQposDim =
R"doc(
Get the dimension of the move group qpos.

:return: dimension of the move group qpos)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getUserJointNames =
R"doc(
Get the joint names that the user has provided for planning.

:return: list of joint names of the user)doc";

static const char *__doc_mplib_ArticulatedModelTpl_getUserLinkNames =
R"doc(
Get the link names that the user has provided for planning.

:return: list of link names of the user)doc";

static const char *__doc_mplib_ArticulatedModelTpl_setBasePose =
R"doc(
Set the base pose of the robot and update all collision links in the
``FCLModel``.

:param pose: base pose of the robot)doc";

static const char *__doc_mplib_ArticulatedModelTpl_setMoveGroup =
R"doc(
Set the move group, i.e. the chain ending in end effector for which to compute
the forward kinematics for all subsequent queries.

:param end_effector: name of the end effector link)doc";

static const char *__doc_mplib_ArticulatedModelTpl_setMoveGroup_2 =
R"doc(
Set the move group but we have multiple end effectors in a chain. I.e., Base -->
EE1 --> EE2 --> ... --> EEn

:param end_effectors: list of links extending to the end effector)doc";

static const char *__doc_mplib_ArticulatedModelTpl_setQpos =
R"doc(
Set the current joint positions and update all collision links in the
``FCLModel``.

:param qpos: current qpos of all active joints or just the move group joints
:param full: whether to set the full qpos or just the move group qpos. If full
    is ``False``, we will pad the missing joints with current known qpos. The
    default is ``False``)doc";

static const char *__doc_mplib_ArticulatedModelTpl_updateSRDF =
R"doc(
Update the SRDF file to disable self-collisions.

:param srdf: path to SRDF file, can be relative to the current working directory)doc";

/* ----- Begin of custom docstring section ----- */

static const char *__doc_mplib_ArticulatedModelTpl_name =
R"doc(Name of the articulated model)doc";

static const char *__doc_mplib_ArticulatedModelTpl_qpos =
R"doc(Current qpos of all active joints)doc";

static const char *__doc_mplib_ArticulatedModelTpl_base_pose =
R"doc(The base (root) pose of the robot)doc";

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
