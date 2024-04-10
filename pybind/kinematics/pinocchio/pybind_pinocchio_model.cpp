#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/kinematics/pinocchio/pinocchio_model.h"
#include "mplib/kinematics/pinocchio/pinocchio_model.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib::kinematics::pinocchio {

using PinocchioModel = PinocchioModelTpl<S>;

void build_pypinocchio_model(py::module &m) {
  auto PyPinocchioModel = py::class_<PinocchioModel, std::shared_ptr<PinocchioModel>>(
      m, "PinocchioModel", DOC(mplib, kinematics, pinocchio, PinocchioModelTpl));

  PyPinocchioModel
      .def(py::init<const std::string &, const Vector3<S> &, bool>(),
           py::arg("urdf_filename"), py::kw_only(),
           py::arg("gravity") = Vector3<S> {0, 0, -9.81}, py::arg("verbose") = false,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, PinocchioModelTpl))

      .def_static(
          "create_from_urdf_string",
          [](const std::string &urdf_string, const Vector3<S> &gravity, bool verbose) {
            std::shared_ptr<PinocchioModel> pinocchio_model =
                PinocchioModel::createFromURDFString(urdf_string, gravity, verbose);
            return pinocchio_model;
          },
          py::arg("urdf_string"), py::kw_only(),
          py::arg("gravity") = Vector3<S> {0, 0, -9.81}, py::arg("verbose") = false,
          DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, createFromURDFString))

      .def("get_leaf_links", &PinocchioModel::getLeafLinks,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getLeafLinks))
      .def("get_adjacent_links", &PinocchioModel::getAdjacentLinks,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getAdjacentLinks))

      .def("set_link_order", &PinocchioModel::setLinkOrder, py::arg("names"),
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, setLinkOrder))
      .def("set_joint_order", &PinocchioModel::setJointOrder, py::arg("names"),
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, setJointOrder))

      .def("get_link_names", &PinocchioModel::getLinkNames, py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getLinkNames))
      .def("get_joint_names", &PinocchioModel::getJointNames, py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointNames))

      .def("get_joint_id", &PinocchioModel::getJointId, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointId))
      .def("get_joint_ids", &PinocchioModel::getJointIds, py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointIds))
      .def("get_joint_type", &PinocchioModel::getJointType, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointType))
      .def("get_joint_types", &PinocchioModel::getJointTypes, py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointTypes))
      .def("get_joint_dim", &PinocchioModel::getJointDim, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointDim))
      .def("get_joint_dims", &PinocchioModel::getJointDims, py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointDims))
      .def("get_joint_limit", &PinocchioModel::getJointLimit, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointLimit))
      .def("get_joint_limits", &PinocchioModel::getJointLimits, py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointLimits))
      .def("get_joint_parent", &PinocchioModel::getJointParent, py::arg("index"),
           py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointParent))
      .def("get_joint_parents", &PinocchioModel::getJointParents,
           py::arg("user") = true,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointParents))
      .def("print_frames", &PinocchioModel::printFrames,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, printFrames))

      .def("get_chain_joint_name", &PinocchioModel::getChainJointName,
           py::arg("end_effector"),
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getChainJointName))
      .def("get_chain_joint_index", &PinocchioModel::getChainJointIndex,
           py::arg("end_effector"),
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getChainJointIndex))

      .def("get_random_configuration", &PinocchioModel::getRandomConfiguration,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getRandomConfiguration))
      .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics,
           py::arg("qpos"),
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl,
               computeForwardKinematics))
      .def(
          "get_link_pose",
          [](const PinocchioModel &self, size_t index) {
            return Pose<S>(self.getLinkPose(index));
          },
          py::arg("index"),
          DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getLinkPose))
      //.def(
      //    "get_joint_pose",
      //    [](const PinocchioModel &self, size_t index) {
      //      return Pose<S>(self.getJointPose(index));
      //    },
      //    py::arg("index"),
      //    DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getJointPose))

      .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian,
           py::arg("qpos"),
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, computeFullJacobian))
      .def("get_link_jacobian", &PinocchioModel::getLinkJacobian, py::arg("index"),
           py::arg("local") = false,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, getLinkJacobian))
      .def("compute_single_link_jacobian", &PinocchioModel::computeSingleLinkJacobian,
           py::arg("qpos"), py::arg("index"), py::arg("local") = false,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl,
               computeSingleLinkJacobian))

      .def("compute_IK_CLIK", &PinocchioModel::computeIKCLIK, py::arg("index"),
           py::arg("pose"), py::arg("q_init"), py::arg("mask") = std::vector<bool>(),
           py::arg("eps") = 1e-5, py::arg("max_iter") = 1000, py::arg("dt") = 1e-1,
           py::arg("damp") = 1e-12,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, computeIKCLIK))
      .def("compute_IK_CLIK_JL", &PinocchioModel::computeIKCLIKJL, py::arg("index"),
           py::arg("pose"), py::arg("q_init"), py::arg("q_min"), py::arg("q_max"),
           py::arg("eps") = 1e-5, py::arg("max_iter") = 1000, py::arg("dt") = 1e-1,
           py::arg("damp") = 1e-12,
           DOC(mplib, kinematics, pinocchio, PinocchioModelTpl, computeIKCLIKJL));
}

}  // namespace mplib::kinematics::pinocchio
