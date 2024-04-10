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

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl =
R"doc(Pinocchio model of an articulation

See https://github.com/stack-of-tasks/pinocchio)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_PinocchioModelTpl =
R"doc(
Construct a Pinocchio model from the given URDF file.

:param urdf_filename: path to the URDF file
:param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
:param verbose: print debug information. Default: ``False``.)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_PinocchioModelTpl_2 =
R"doc(
Construct a Pinocchio model from the given URDF file.

:param urdf_model: a urdf tree as urdf::ModelInterfaceSharedPtr
:param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
:param verbose: print debug information. Default: ``False``.)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_computeForwardKinematics =
R"doc(
Compute forward kinematics for the given joint configuration.

If you want the result you need to call ``get_link_pose()``

:param qpos: joint configuration. Needs to be full configuration, not just the
    movegroup joints.)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_computeFullJacobian =
R"doc(
Compute the full jacobian for the given joint configuration. Note you need to
call computeForwardKinematics() first. If you want the result you need to call
``get_link_jacobian()``

:param qpos: joint configuration. Needs to be full configuration, not just the
    movegroup joints.)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_computeIKCLIK =
R"doc(
Compute the inverse kinematics using close loop inverse kinematics.

:param index: index of the link (in the order you passed to the constructor or
    the default order)
:param pose: desired pose of the link
:param q_init: initial joint configuration
:param mask: if the value at a given index is ``True``, the joint is *not* used
    in the IK
:param eps: tolerance for the IK
:param max_iter: maximum number of iterations
:param dt: time step for the CLIK
:param damp: damping for the CLIK
:return: joint configuration)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_computeIKCLIKJL =
R"doc(
The same as ``compute_IK_CLIK()`` but with it clamps the joint configuration to
the given limits.

:param index: index of the link (in the order you passed to the constructor or
    the default order)
:param pose: desired pose of the link
:param q_init: initial joint configuration
:param q_min: minimum joint configuration
:param q_max: maximum joint configuration
:param eps: tolerance for the IK
:param max_iter: maximum number of iterations
:param dt: time step for the CLIK
:param damp: damping for the CLIK
:return: joint configuration)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_computeSingleLinkJacobian =
R"doc(
Compute the jacobian of the given link. Note you need to call
computeForwardKinematics() first.

:param qpos: joint configuration. Needs to be full configuration, not just the
    movegroup joints.
:param index: index of the link (in the order you passed to the constructor or
    the default order)
:param local: if ``True`` return the jacobian w.r.t. the instantaneous local
    frame of the link
:return: 6 x n jacobian of the link)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_createFromURDFString =
R"doc(
Constructs a PinocchioModel from URDF string

:param urdf_string: URDF string (without visual/collision elements for links)
:param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
:param verbose: print debug information. Default: ``False``.
:return: a unique_ptr to PinocchioModel)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getAdjacentLinks =
R"doc(
Get the all adjacent link names.

:return: adjacent link names as a set of pairs of strings)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getChainJointIndex =
R"doc(
Get the joint indices of the joints in the chain from the root to the given
link.

:param index: index of the link (in the order you passed to the constructor or
    the default order)
:return: joint indices of the joints in the chain)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getChainJointName =
R"doc(
Get the joint names of the joints in the chain from the root to the given link.

:param index: index of the link (in the order you passed to the constructor or
    the default order)
:return: joint names of the joints in the chain)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getData =
R"doc(
Get the underlying pinocchio::Data)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointDim =
R"doc(
Get the dimension of the joint with the given index.

:param index: joint index to query
:param user: if ``True``, the joint index follows the order you passed to the
    constructor or the default order
:return: dimension of the joint with the given index)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointDims =
R"doc(
Get the dimension of all the joints. Again, Pinocchio might split a joint into
multiple joints.

:param user: if ``True``, we get the dimension of the joints in the order you
    passed to the constructor or the default order
:return: dimention of the joints)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointId =
R"doc(
Get the id of the joint with the given index.

:param index: joint index to query
:param user: if ``True``, the joint index follows the order you passed to the
    constructor or the default order
:return: id of the joint with the given index)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointIds =
R"doc(
Get the id of all the joints. Again, Pinocchio might split a joint into multiple
joints.

:param user: if ``True``, we get the id of the joints in the order you passed to
    the constructor or the default order
:return: id of the joints)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointLimit =
R"doc(
Get the limit of the joint with the given index.

:param index: joint index to query
:param user: if ``True``, the joint index follows the order you passed to the
    constructor or the default order
:return: limit of the joint with the given index)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointLimits =
R"doc(
Get the limit of all the joints. Again, Pinocchio might split a joint into
multiple joints.

:param user: if ``True``, we get the limit of the joints in the order you passed
    to the constructor or the default order
:return: limit of the joints)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointNames =
R"doc(
Get the name of all the joints. Again, Pinocchio might split a joint into
multiple joints.

:param user: if ``True``, we get the name of the joints in the order you passed
    to the constructor or the default order
:return: name of the joints)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointParent =
R"doc(
Get the parent of the joint with the given index.

:param index: joint index to query
:param user: if ``True``, the joint index follows the order you passed to the
    constructor or the default order
:return: parent of the joint with the given index)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointParents =
R"doc(
Get the parent of all the joints. Again, Pinocchio might split a joint into
multiple joints.

:param user: if ``True``, we get the parent of the joints in the order you
    passed to the constructor or the default order
:return: parent of the joints)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointPose =
R"doc(
)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointType =
R"doc(
Get the type of the joint with the given index.

:param index: joint index to query
:param user: if ``True``, the joint index follows the order you passed to the
    constructor or the default order
:return: type of the joint with the given index)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getJointTypes =
R"doc(
Get the type of all the joints. Again, Pinocchio might split a joint into
multiple joints.

:param user: if ``True``, we get the type of the joints in the order you passed
    to the constructor or the default order
:return: type of the joints)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getLeafLinks =
R"doc(
Get the leaf links (links without child) of the kinematic tree.

:return: list of leaf links)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getLinkJacobian =
R"doc(
Get the jacobian of the given link. You must call ``compute_full_jacobian()``
first.

:param index: index of the link (in the order you passed to the constructor or
    the default order)
:param local: if ``True``, the jacobian is expressed in the local frame of the
    link, otherwise it is expressed in the world frame
:return: 6 x n jacobian of the link)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getLinkNames =
R"doc(
Get the name of all the links.

:param user: if ``True``, we get the name of the links in the order you passed
    to the constructor or the default order
:return: name of the links)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getLinkPose =
R"doc(
Get the pose of the given link in robot's base (root) frame.

:param index: index of the link (in the order you passed to the constructor or
    the default order)
:return: pose of the link in robot's base (root) frame.)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getModel =
R"doc(
Get the underlying pinocchio::Model)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_getRandomConfiguration =
R"doc(
Get a random configuration.

:return: random joint configuration)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_printFrames =
R"doc(
Frame is a Pinocchio internal data type which is not supported outside this
class.)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_setJointOrder =
R"doc(
Pinocchio might have a different joint order or it might add additional joints.

If you do not pass the the list of joint names, the default order might not be
the one you want.

:param names: list of joint names in the order you want)doc";

static const char *__doc_mplib_kinematics_pinocchio_PinocchioModelTpl_setLinkOrder =
R"doc(
Pinocchio might have a different link order or it might add additional links.

If you do not pass the the list of link names, the default order might not be
the one you want.

:param names: list of link names in the order you want)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
