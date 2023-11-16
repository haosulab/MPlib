#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include "../src/articulated_model.h"

namespace py = pybind11;

#ifdef USE_SINGLE
using DATATYPE = float;
#else
using DATATYPE = double;
#endif

using ArticulatedModel = ArticulatedModelTpl<DATATYPE>;


void build_pyarticulation(py::module &m_all) {
    auto m = m_all.def_submodule("articulation");
    auto PyArticulatedModel = py::class_<ArticulatedModel, std::shared_ptr<ArticulatedModel>>(m, "ArticulatedModel");

    PyArticulatedModel
            .def(py::init<std::string const &, std::string const &, Eigen::Matrix<DATATYPE, 3, 1>, std::vector<std::string>
                 const &, std::vector<std::string> const &, bool const &, bool const&>(),
                 py::arg("urdf_filename"), py::arg("srdf_filename"), py::arg("gravity"), py::arg("joint_names"),
                 py::arg("link_names"), py::arg("verbose") = true, py::arg("convex") = false)
            .def("get_pinocchio_model", &ArticulatedModel::getPinocchioModel)
            .def("get_fcl_model", &ArticulatedModel::getFCLModel)
            .def("set_move_group", py::overload_cast<std::string const &>(&ArticulatedModel::setMoveGroup),
                 py::arg("end_effector"))
            .def("set_move_group", py::overload_cast<std::vector<std::string> const &>(&ArticulatedModel::setMoveGroup),
                 py::arg("end_effectors"))
            .def("get_move_group_joint_indices", &ArticulatedModel::getMoveGroupJointIndices)
            .def("get_move_group_joint_names", &ArticulatedModel::getMoveGroupJointName)
            .def("get_user_joint_names", &ArticulatedModel::getUserJointNames)
            .def("get_user_link_names", &ArticulatedModel::getUserLinkNames)
            .def("get_move_group_end_effectors", &ArticulatedModel::getMoveGroupEndEffectors)
            .def("get_qpos", &ArticulatedModel::getQpos)
            .def("get_qpos_dim", &ArticulatedModel::getQposDim)
            .def("set_qpos", &ArticulatedModel::setQpos, py::arg("qpos"), py::arg("full") = false)
            .def("update_SRDF", &ArticulatedModel::updateSRDF, py::arg("SRDF"))
            .def("set_base_pose", &ArticulatedModel::setBasePose, py::arg("pose"))
            .def("get_base_pose", &ArticulatedModel::getBasePose);
}
