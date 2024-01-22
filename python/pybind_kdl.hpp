#pragma once

#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/kdl_model.h"
#include "mplib/kdl_model.h"
#include "mplib/macros_utils.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

using KDLModel = KDLModelTpl<S>;
DEFINE_TEMPLATE_EIGEN(S)

void build_pykdl(py::module &m_all) {
  auto m = m_all.def_submodule("kdl");
  auto PyKDLModel =
      py::class_<KDLModel, std::shared_ptr<KDLModel>>(m, "KDLModel", DOC(KDLModelTpl));

  PyKDLModel
      .def(py::init<const std::string &, const std::vector<std::string> &,
                    const std::vector<std::string> &, const bool &>(),
           py::arg("urdf_filename"), py::arg("joint_names"), py::arg("link_names"),
           py::arg("verbose"), DOC(KDLModelTpl, KDLModelTpl))
      .def("get_tree_root_name", &KDLModel::getTreeRootName,
           DOC(KDLModelTpl, getTreeRootName))
      .def("chain_IK_LMA", &KDLModel::chainIKLMA, py::arg("index"), py::arg("q_init"),
           py::arg("goal_pose"), DOC(KDLModelTpl, chainIKLMA))
      .def("chain_IK_NR", &KDLModel::chainIKNR, py::arg("index"), py::arg("q_init"),
           py::arg("goal_pose"), DOC(KDLModelTpl, chainIKNR))
      .def("chain_IK_NR_JL", &KDLModel::chainIKNRJL, py::arg("index"),
           py::arg("q_init"), py::arg("goal_pose"), py::arg("q_min"), py::arg("q_max"),
           DOC(KDLModelTpl, chainIKNRJL))
      .def("tree_IK_NR_JL", &KDLModel::TreeIKNRJL, py::arg("endpoints"),
           py::arg("q_init"), py::arg("goal_poses"), py::arg("q_min"), py::arg("q_max"),
           DOC(KDLModelTpl, TreeIKNRJL));
}
