#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include "../src/pinocchio_model.h"
#include "../src/macros_utils.hpp"

namespace py = pybind11;

#ifdef USE_SINGLE
using DATATYPE=float;
#else
using DATATYPE = double;
#endif

using PinocchioModel = PinocchioModelTpl<DATATYPE>;
DEFINE_TEMPLATE_EIGEN(DATATYPE)

std::string pinocchio_ctor_doc = R"(
     Args:
          urdf_filename: path to the urdf file
          gravity: gravity vector
          verbose: print debug information
     Returns:
          PinocchioModel object)";

std::string set_joint_order_doc = R"(
     Pinocchio might have a different joint order or it might add additional joints.
     If you do not pass the the list of joint names, the default order might not be the one you want.
     Args:
          names: list of joint names in the order you want)";

std::string set_link_order_doc = R"(
     Pinocchio might have a different link order or it might add additional links.
     If you do not pass the the list of link names, the default order might not be the one you want.
     Args:
          names: list of link names in the order you want)";

std::string compute_forward_kinematics_doc = R"(
     Compute forward kinematics for the given joint configuration.
     Args:
          qpos: joint configuration. Needs to be full configuration, not just the movegroup joints.
     Returns:
          None. If you want the result you need to call get_link_pose)";
          
std::string get_link_pose_doc = R"(
     Get the pose of the given link.
     Args:
          index: index of the link (in the order you passed to the constructor or the default order)
     Returns:
          pose of the link [x,y,z,qw,qx,qy,qz])";

std::string get_random_configuration_doc = R"(
     Get a random configuration.
     Returns:
          random joint configuration)";

std::string compute_full_jacobian_doc = R"(
     Compute the full jacobian for the given joint configuration.
     Args:
          qpos: joint configuration. Needs to be full configuration, not just the movegroup joints.
     Returns:
          None. If you want the result you need to call get_link_jacobian)";

std::string get_link_jacobian_doc = R"(
     Get the jacobian of the given link.
     Args:
          index: index of the link (in the order you passed to the constructor or the default order)
          local: if True, the jacobian is expressed in the local frame of the link, otherwise it is expressed in the world frame
     Returns:
          6 x n jacobian of the link)";

std::string compute_single_link_local_jacobian_doc = R"(
     Compute the jacobian of the given link.
     Args:
          qpos: joint configuration. Needs to be full configuration, not just the movegroup joints.
          index: index of the link (in the order you passed to the constructor or the default order)
     Returns:
          6 x n jacobian of the link)";

std::string compute_IK_CLIK_doc = R"(
     Compute the inverse kinematics using close loop inverse kinematics.
     Args:
          index: index of the link (in the order you passed to the constructor or the default order)
          pose: desired pose of the link [x,y,z,qw,qx,qy,qz]
          q_init: initial joint configuration
          mask: mask of the joints to use for the IK
          eps: tolerance for the IK
          maxIter: maximum number of iterations
          dt: time step for the CLIK
          damp: damping for the CLIK
     Returns:
          joint configuration)";

std::string compute_IK_CLIK_JL_doc = R"(
     The same as compute_IK_CLIK but with it clamps the joint configuration to the given limits.
     Args:
          index: index of the link (in the order you passed to the constructor or the default order)
          pose: desired pose of the link [x,y,z,qw,qx,qy,qz]
          q_init: initial joint configuration
          q_min: minimum joint configuration
          q_max: maximum joint configuration
          eps: tolerance for the IK
          maxIter: maximum number of iterations
          dt: time step for the CLIK
          damp: damping for the CLIK
     Returns:
          joint configuration)";

std::string get_leaf_link_doc = R"(
     Get the leaf links (links without child) of the kinematic tree.
     Returns:
          list of leaf links)";

std::string get_joint_ids_doc = R"(
     Get the id of the all the joints. Again, Pinocchio might split a joint into multiple joints.
     Args:
          user: if True, we get the id of the joints in the order you passed to the constructor or the default order
     Returns:
          ids of the joint)";

std::string get_parents_doc = R"(
     Get the parent of the all the joints. Again, Pinocchio might split a joint into multiple joints.
     Args:
          user: if True, we get the parent of the joints in the order you passed to the constructor or the default order
     Returns:
          parents of the joints)";

std::string get_chain_joint_name_doc = R"(
     Get the joint names of the joints in the chain from the root to the given link.
     Args:
          index: index of the link (in the order you passed to the constructor or the default order)
     Returns:
          joint names of the joints in the chain)";

std::string get_chain_joint_index_doc = R"(
     Get the joint indices of the joints in the chain from the root to the given link.
     Args:
          index: index of the link (in the order you passed to the constructor or the default order)
     Returns:
          joint indices of the joints in the chain)";

void build_pypinocchio(py::module &m_all) {
    auto m = m_all.def_submodule("pinocchio");
    auto PyPinocchioModel = py::class_<PinocchioModel, std::shared_ptr<PinocchioModel>>(m, "PinocchioModel");

    PyPinocchioModel
            .def(py::init<std::string const &, Vector3, bool>(),
                 py::arg("urdf_filename"),
                 py::arg("gravity")=Vector3(0, 0, -9.81),
                 py::arg("verbose")=true,
                 pinocchio_ctor_doc.c_str())
            .def("set_joint_order", &PinocchioModel::setJointOrder, py::arg("names"), set_joint_order_doc.c_str())
            .def("set_link_order", &PinocchioModel::setLinkOrder, py::arg("names"), set_link_order_doc.c_str())
            .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics, py::arg("qpos"), compute_forward_kinematics_doc.c_str())
            .def("get_link_pose", &PinocchioModel::getLinkPose, py::arg("index"), get_link_pose_doc.c_str())
            //.def("get_joint_pose", &PinocchioModel::getJointPose, py::arg("index"))
            .def("get_random_configuration", &PinocchioModel::getRandomConfiguration, get_random_configuration_doc.c_str())
            .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian, py::arg("qpos"), compute_full_jacobian_doc.c_str())
            .def("get_link_jacobian", &PinocchioModel::getLinkJacobian, py::arg("index"), py::arg("local") = false, get_link_jacobian_doc.c_str())
            .def("compute_single_link_local_jacobian", &PinocchioModel::computeSingleLinkLocalJacobian, py::arg("qpos"),
                 py::arg("index"), compute_single_link_local_jacobian_doc.c_str())
            .def("compute_IK_CLIK", &PinocchioModel::computeIKCLIK,
                 py::arg("index"), py::arg("pose"), py::arg("q_init"), py::arg("mask") = std::vector<bool>(),  py::arg("eps") = 1e-5,
                 py::arg("maxIter") = 1000, py::arg("dt") = 1e-1, py::arg("damp") = 1e-12, compute_IK_CLIK_doc.c_str())
            .def("compute_IK_CLIK_JL", &PinocchioModel::computeIKCLIKJL,
                 py::arg("index"), py::arg("pose"), py::arg("q_init"), py::arg("q_min"), py::arg("q_max"), py::arg("eps") = 1e-5,
                 py::arg("maxIter") = 1000, py::arg("dt") = 1e-1, py::arg("damp") = 1e-12, compute_IK_CLIK_JL_doc.c_str())
            .def("get_joint_names", &PinocchioModel::getJointNames, py::arg("user")=true)
            .def("get_link_names", &PinocchioModel::getLinkNames, py::arg("user")=true)
            .def("get_leaf_links", &PinocchioModel::getLeafLinks, get_leaf_link_doc.c_str())
            .def("get_joint_dim", &PinocchioModel::getJointDim, py::arg("index"), py::arg("user")=true)
            .def("get_joint_dims", &PinocchioModel::getJointDims, py::arg("user")=true)
            .def("get_joint_ids", &PinocchioModel::getJointIds, py::arg("user")=true, get_joint_ids_doc.c_str())
            .def("get_parents", &PinocchioModel::getParents, py::arg("user")=true, get_parents_doc.c_str())
            .def("get_joint_types", &PinocchioModel::getJointTypes, py::arg("user")=true)
            .def("get_joint_limits", &PinocchioModel::getJointLimits, py::arg("user")=true)
            .def("print_frames", &PinocchioModel::printFrames)
            .def("get_chain_joint_name", &PinocchioModel::getChainJointName, py::arg("end_effector"), get_chain_joint_name_doc.c_str())
            .def("get_chain_joint_index", &PinocchioModel::getChainJointIndex, py::arg("end_effector"), get_chain_joint_index_doc.c_str());
}