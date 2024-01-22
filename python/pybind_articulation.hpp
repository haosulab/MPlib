#pragma once

#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../src/articulated_model.h"
#include "docstring/articulated_model.h"

namespace py = pybind11;

using DATATYPE = double;

using ArticulatedModel = ArticulatedModelTpl<DATATYPE>;

void build_pyarticulation(py::module &m_all) {
  auto m = m_all.def_submodule(
      "articulation", "articulated model submodule, i.e. models with moving parts");
  auto PyArticulatedModel =
      py::class_<ArticulatedModel, std::shared_ptr<ArticulatedModel>>(
          m, "ArticulatedModel", DOC(ArticulatedModelTpl));

  PyArticulatedModel
      .def(py::init<const std::string &, const std::string &,
                    Eigen::Matrix<DATATYPE, 3, 1>, const std::vector<std::string> &,
                    const std::vector<std::string> &, const bool &, const bool &>(),
           py::arg("urdf_filename"), py::arg("srdf_filename"), py::arg("gravity"),
           py::arg("joint_names"), py::arg("link_names"), py::arg("verbose") = true,
           py::arg("convex") = false, DOC(ArticulatedModelTpl, ArticulatedModelTpl))
      .def("get_pinocchio_model", &ArticulatedModel::getPinocchioModel,
           DOC(ArticulatedModelTpl, getPinocchioModel))
      .def("get_fcl_model", &ArticulatedModel::getFCLModel,
           DOC(ArticulatedModelTpl, getFCLModel))
      .def("set_move_group",
           py::overload_cast<const std::string &>(&ArticulatedModel::setMoveGroup),
           py::arg("end_effector"), DOC(ArticulatedModelTpl, setMoveGroup))
      .def("set_move_group",
           py::overload_cast<const std::vector<std::string> &>(
               &ArticulatedModel::setMoveGroup),
           py::arg("end_effectors"), DOC(ArticulatedModelTpl, setMoveGroup, 2))
      .def("set_base_pose", &ArticulatedModel::setBasePose, py::arg("pose"),
           DOC(ArticulatedModelTpl, setBasePose))
      .def("get_base_pose", &ArticulatedModel::getBasePose,
           DOC(ArticulatedModelTpl, getBasePose))
      .def("get_move_group_joint_indices", &ArticulatedModel::getMoveGroupJointIndices,
           DOC(ArticulatedModelTpl, getMoveGroupJointIndices))
      .def("get_move_group_joint_names", &ArticulatedModel::getMoveGroupJointName,
           DOC(ArticulatedModelTpl, getMoveGroupJointName))
      .def("get_user_joint_names", &ArticulatedModel::getUserJointNames,
           DOC(ArticulatedModelTpl, getUserJointNames))
      .def("get_user_link_names", &ArticulatedModel::getUserLinkNames,
           DOC(ArticulatedModelTpl, getUserLinkNames))
      .def("get_move_group_end_effectors", &ArticulatedModel::getMoveGroupEndEffectors,
           DOC(ArticulatedModelTpl, getMoveGroupEndEffectors))
      .def("get_qpos", &ArticulatedModel::getQpos, DOC(ArticulatedModelTpl, getQpos))
      .def("get_move_group_qpos_dim", &ArticulatedModel::getQposDim,
           DOC(ArticulatedModelTpl, getQposDim))
      .def("set_qpos", &ArticulatedModel::setQpos, py::arg("qpos"),
           py::arg("full") = false, DOC(ArticulatedModelTpl, setQpos))
      .def("update_SRDF", &ArticulatedModel::updateSRDF, py::arg("SRDF"),
           DOC(ArticulatedModelTpl, updateSRDF));
}
