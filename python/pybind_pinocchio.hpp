#pragma once

#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/pinocchio_model.h"
#include "mplib/macros_utils.h"
#include "mplib/pinocchio_model.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

using PinocchioModel = PinocchioModelTpl<S>;
DEFINE_TEMPLATE_EIGEN(S)

void build_pypinocchio(py::module &m_all) {
  auto m = m_all.def_submodule("pinocchio");
  auto PyPinocchioModel = py::class_<PinocchioModel, std::shared_ptr<PinocchioModel>>(
      m, "PinocchioModel", DOC(PinocchioModelTpl));

  PyPinocchioModel
      .def(py::init<const std::string &, Vector3, bool>(), py::arg("urdf_filename"),
           py::arg("gravity") = Vector3(0, 0, -9.81), py::arg("verbose") = true,
           DOC(PinocchioModelTpl, PinocchioModelTpl, 2))
      .def("set_joint_order", &PinocchioModel::setJointOrder, py::arg("names"),
           DOC(PinocchioModelTpl, setJointOrder))
      .def("set_link_order", &PinocchioModel::setLinkOrder, py::arg("names"),
           DOC(PinocchioModelTpl, setLinkOrder))
      .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics,
           py::arg("qpos"), DOC(PinocchioModelTpl, computeForwardKinematics))
      .def("get_link_pose", &PinocchioModel::getLinkPose, py::arg("index"),
           DOC(PinocchioModelTpl, getLinkPose))
      //.def("get_joint_pose", &PinocchioModel::getJointPose, py::arg("index"))
      .def("get_random_configuration", &PinocchioModel::getRandomConfiguration,
           DOC(PinocchioModelTpl, getRandomConfiguration))
      .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian,
           py::arg("qpos"), DOC(PinocchioModelTpl, computeFullJacobian))
      .def("get_link_jacobian", &PinocchioModel::getLinkJacobian, py::arg("index"),
           py::arg("local") = false, DOC(PinocchioModelTpl, getLinkJacobian))
      .def("compute_single_link_jacobian", &PinocchioModel::computeSingleLinkJacobian,
           py::arg("qpos"), py::arg("index"), py::arg("local") = false,
           DOC(PinocchioModelTpl, computeSingleLinkJacobian))
      .def("compute_IK_CLIK", &PinocchioModel::computeIKCLIK, py::arg("index"),
           py::arg("pose"), py::arg("q_init"), py::arg("mask") = std::vector<bool>(),
           py::arg("eps") = 1e-5, py::arg("maxIter") = 1000, py::arg("dt") = 1e-1,
           py::arg("damp") = 1e-12, DOC(PinocchioModelTpl, computeIKCLIK))
      .def("compute_IK_CLIK_JL", &PinocchioModel::computeIKCLIKJL, py::arg("index"),
           py::arg("pose"), py::arg("q_init"), py::arg("q_min"), py::arg("q_max"),
           py::arg("eps") = 1e-5, py::arg("maxIter") = 1000, py::arg("dt") = 1e-1,
           py::arg("damp") = 1e-12, DOC(PinocchioModelTpl, computeIKCLIKJL))
      .def("get_joint_names", &PinocchioModel::getJointNames, py::arg("user") = true,
           DOC(PinocchioModelTpl, getJointNames))
      .def("get_link_names", &PinocchioModel::getLinkNames, py::arg("user") = true,
           DOC(PinocchioModelTpl, getLinkNames))
      .def("get_leaf_links", &PinocchioModel::getLeafLinks,
           DOC(PinocchioModelTpl, getLeafLinks))
      .def("get_joint_dim", &PinocchioModel::getJointDim, py::arg("index"),
           py::arg("user") = true, DOC(PinocchioModelTpl, getJointDim))
      .def("get_joint_dims", &PinocchioModel::getJointDims, py::arg("user") = true,
           DOC(PinocchioModelTpl, getJointDims))
      .def("get_joint_ids", &PinocchioModel::getJointIds, py::arg("user") = true,
           DOC(PinocchioModelTpl, getJointIds))
      .def("get_parents", &PinocchioModel::getParents, py::arg("user") = true,
           DOC(PinocchioModelTpl, getParents))
      .def("get_joint_types", &PinocchioModel::getJointTypes, py::arg("user") = true,
           DOC(PinocchioModelTpl, getJointTypes))
      .def("get_joint_limits", &PinocchioModel::getJointLimits, py::arg("user") = true,
           DOC(PinocchioModelTpl, getJointLimits))
      .def("print_frames", &PinocchioModel::printFrames,
           DOC(PinocchioModelTpl, printFrames))
      .def("get_chain_joint_name", &PinocchioModel::getChainJointName,
           py::arg("end_effector"), DOC(PinocchioModelTpl, getChainJointName))
      .def("get_chain_joint_index", &PinocchioModel::getChainJointIndex,
           py::arg("end_effector"), DOC(PinocchioModelTpl, getChainJointIndex));
}
