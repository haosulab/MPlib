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

using DATATYPE = double;

using PinocchioModel = PinocchioModelTpl<DATATYPE>;
DEFINE_TEMPLATE_EIGEN(DATATYPE)


void build_pypinocchio(py::module &m_all) {
    auto m = m_all.def_submodule("pinocchio");
    auto PyPinocchioModel = py::class_<PinocchioModel, std::shared_ptr<PinocchioModel>>(m, "PinocchioModel");

    PyPinocchioModel
            .def(py::init<std::string const &, Vector3, bool>(),
                 py::arg("urdf_filename"), py::arg("gravity") = Vector3(0, 0, -9.81), py::arg("verbose") = true)
            .def("set_joint_order", &PinocchioModel::setJointOrder, py::arg("names"))
            .def("set_link_order", &PinocchioModel::setLinkOrder, py::arg("names"))
            .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics, py::arg("qpos"))
            .def("get_link_pose", &PinocchioModel::getLinkPose, py::arg("index"))
            //.def("get_joint_pose", &PinocchioModel::getJointPose, py::arg("index"))
            .def("get_random_configuration", &PinocchioModel::getRandomConfiguration)
            .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian, py::arg("qpos"))
            .def("get_link_jacobian", &PinocchioModel::getLinkJacobian, py::arg("index"), py::arg("local") = false)
            .def("compute_single_link_local_jacobian", &PinocchioModel::computeSingleLinkLocalJacobian, py::arg("qpos"),
                 py::arg("index"))
            .def("compute_IK_CLIK", &PinocchioModel::computeIKCLIK,
                 py::arg("index"), py::arg("pose"), py::arg("q_init"), py::arg("mask") = std::vector<bool>(),  py::arg("eps") = 1e-5,
                 py::arg("maxIter") = 1000, py::arg("dt") = 1e-1, py::arg("damp") = 1e-12)
            .def("compute_IK_CLIK_JL", &PinocchioModel::computeIKCLIKJL,
                 py::arg("index"), py::arg("pose"), py::arg("q_init"), py::arg("q_min"), py::arg("q_max"), py::arg("eps") = 1e-5,
                 py::arg("maxIter") = 1000, py::arg("dt") = 1e-1, py::arg("damp") = 1e-12)
            .def("get_joint_names", &PinocchioModel::getJointNames, py::arg("user")=true)
            .def("get_link_names", &PinocchioModel::getLinkNames, py::arg("user")=true)
            .def("get_leaf_links", &PinocchioModel::getLeafLinks)
            .def("get_joint_dim", &PinocchioModel::getJointDim, py::arg("index"), py::arg("user")=true)
            .def("get_joint_dims", &PinocchioModel::getJointDims, py::arg("user")=true)
            .def("get_joint_ids", &PinocchioModel::getJointIds, py::arg("user")=true)
            .def("get_parents", &PinocchioModel::getParents, py::arg("user")=true)
            .def("get_joint_types", &PinocchioModel::getJointTypes, py::arg("user")=true)
            .def("get_joint_limits", &PinocchioModel::getJointLimits, py::arg("user")=true)
            .def("print_frames", &PinocchioModel::printFrames)
            .def("get_chain_joint_name", &PinocchioModel::getChainJointName)
            .def("get_chain_joint_index", &PinocchioModel::getChainJointIndex);
}