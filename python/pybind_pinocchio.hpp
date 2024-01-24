#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/pinocchio_model.h"
#include "mplib/pinocchio_model.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using PinocchioModel = pinocchio::PinocchioModelTpl<S>;

inline void build_pypinocchio(py::module &m_all) {
  auto m = m_all.def_submodule("pinocchio");

  auto PyPinocchioModel = py::class_<PinocchioModel, std::shared_ptr<PinocchioModel>>(
      m, "PinocchioModel", DOC(mplib, pinocchio, PinocchioModelTpl));

  PyPinocchioModel
      .def(py::init<const std::string &, Vector3<S>, bool>(), py::arg("urdf_filename"),
           py::arg("gravity") = Vector3<S>(0, 0, -9.81), py::arg("verbose") = false,
           DOC(mplib, pinocchio, PinocchioModelTpl, PinocchioModelTpl))

      .def("get_leaf_links", &PinocchioModel::getLeafLinks,
           DOC(mplib, pinocchio, PinocchioModelTpl, getLeafLinks))

      .def("set_link_order", &PinocchioModel::setLinkOrder, py::arg("names"),
           DOC(mplib, pinocchio, PinocchioModelTpl, setLinkOrder))
      .def("set_joint_order", &PinocchioModel::setJointOrder, py::arg("names"),
           DOC(mplib, pinocchio, PinocchioModelTpl, setJointOrder))

      .def("get_link_names", &PinocchioModel::getLinkNames, py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getLinkNames))
      .def("get_joint_names", &PinocchioModel::getJointNames, py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointNames))

      .def("get_joint_id", &PinocchioModel::getJointId, py::arg("index"),
           py::arg("user") = true, DOC(mplib, pinocchio, PinocchioModelTpl, getJointId))
      .def("get_joint_ids", &PinocchioModel::getJointIds, py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointIds))
      .def("get_joint_type", &PinocchioModel::getJointType, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointType))
      .def("get_joint_types", &PinocchioModel::getJointTypes, py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointTypes))
      .def("get_joint_dim", &PinocchioModel::getJointDim, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointDim))
      .def("get_joint_dims", &PinocchioModel::getJointDims, py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointDims))
      .def("get_joint_limit", &PinocchioModel::getJointLimit, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointLimit))
      .def("get_joint_limits", &PinocchioModel::getJointLimits, py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointLimits))
      .def("get_joint_parent", &PinocchioModel::getJointParent, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointParent))
      .def("get_joint_parents", &PinocchioModel::getJointParents,
           py::arg("user") = true,
           DOC(mplib, pinocchio, PinocchioModelTpl, getJointParents))
      .def("print_frames", &PinocchioModel::printFrames,
           DOC(mplib, pinocchio, PinocchioModelTpl, printFrames))

      .def("get_chain_joint_name", &PinocchioModel::getChainJointName,
           py::arg("end_effector"),
           DOC(mplib, pinocchio, PinocchioModelTpl, getChainJointName))
      .def("get_chain_joint_index", &PinocchioModel::getChainJointIndex,
           py::arg("end_effector"),
           DOC(mplib, pinocchio, PinocchioModelTpl, getChainJointIndex))

      .def("get_random_configuration", &PinocchioModel::getRandomConfiguration,
           DOC(mplib, pinocchio, PinocchioModelTpl, getRandomConfiguration))
      .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics,
           py::arg("qpos"),
           DOC(mplib, pinocchio, PinocchioModelTpl, computeForwardKinematics))
      .def("get_link_pose", &PinocchioModel::getLinkPose, py::arg("index"),
           DOC(mplib, pinocchio, PinocchioModelTpl, getLinkPose))
      //.def("get_joint_pose", &PinocchioModel::getJointPose, py::arg("index"))

      .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian,
           py::arg("qpos"),
           DOC(mplib, pinocchio, PinocchioModelTpl, computeFullJacobian))
      .def("get_link_jacobian", &PinocchioModel::getLinkJacobian, py::arg("index"),
           py::arg("local") = false,
           DOC(mplib, pinocchio, PinocchioModelTpl, getLinkJacobian))
      .def("compute_single_link_jacobian", &PinocchioModel::computeSingleLinkJacobian,
           py::arg("qpos"), py::arg("index"), py::arg("local") = false,
           DOC(mplib, pinocchio, PinocchioModelTpl, computeSingleLinkJacobian))

      .def("compute_IK_CLIK", &PinocchioModel::computeIKCLIK, py::arg("index"),
           py::arg("pose"), py::arg("q_init"), py::arg("mask") = std::vector<bool>(),
           py::arg("eps") = 1e-5, py::arg("maxIter") = 1000, py::arg("dt") = 1e-1,
           py::arg("damp") = 1e-12,
           DOC(mplib, pinocchio, PinocchioModelTpl, computeIKCLIK))
      .def("compute_IK_CLIK_JL", &PinocchioModel::computeIKCLIKJL, py::arg("index"),
           py::arg("pose"), py::arg("q_init"), py::arg("q_min"), py::arg("q_max"),
           py::arg("eps") = 1e-5, py::arg("maxIter") = 1000, py::arg("dt") = 1e-1,
           py::arg("damp") = 1e-12,
           DOC(mplib, pinocchio, PinocchioModelTpl, computeIKCLIKJL));
}

}  // namespace mplib
