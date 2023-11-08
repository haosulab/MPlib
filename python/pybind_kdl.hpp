#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include "../src/kdl_model.h"
#include "../src/macros_utils.hpp"

namespace py = pybind11;

using DATATYPE = double;

using KDLModel = KDLModelTpl<DATATYPE>;
DEFINE_TEMPLATE_EIGEN(DATATYPE)


void build_pykdl(py::module &m_all) {
    auto m = m_all.def_submodule("kdl");
    auto PyKDLModel = py::class_<KDLModel, std::shared_ptr<KDLModel>>(m, "KDLModel");

    PyKDLModel
            .def(py::init<std::string const &, std::vector<std::string> const &,
             std::vector<std::string> const &, bool const &>(), py::arg("urdf_filename"), py::arg("joint_names"), py::arg("link_names"), py::arg("verbose"))
            .def("get_tree_root_name", &KDLModel::getTreeRootName)
            .def("chain_IK_LMA", &KDLModel::chainIKLMA, py::arg("index"), py::arg("q_init"), py::arg("goal_pose"))
            .def("chain_IK_NR", &KDLModel::chainIKNR, py::arg("index"), py::arg("q_init"), py::arg("goal_pose"))
            .def("chain_IK_NR_JL", &KDLModel::chainIKNRJL, py::arg("index"), py::arg("q_init"), py::arg("goal_pose"), py::arg("q_min"), py::arg("q_max"))
            .def("tree_IK_NR_JL", &KDLModel::TreeIKNRJL, py::arg("endpoints"), py::arg("q_init"), py::arg("goal_poses"), py::arg("q_min"), py::arg("q_max"));
}