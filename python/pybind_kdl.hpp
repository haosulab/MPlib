#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/kdl_model.h"
#include "mplib/kdl_model.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using KDLModel = KDLModelTpl<S>;

inline void build_pykdl(py::module &m_all) {
  auto m = m_all.def_submodule("kdl");
  auto PyKDLModel = py::class_<KDLModel, std::shared_ptr<KDLModel>>(
      m, "KDLModel", DOC(mplib, kdl, KDLModelTpl));

  PyKDLModel
      .def(py::init<const std::string &, const std::vector<std::string> &,
                    const std::vector<std::string> &, const bool &>(),
           py::arg("urdf_filename"), py::arg("joint_names"), py::arg("link_names"),
           py::arg("verbose"), DOC(mplib, kdl, KDLModelTpl, KDLModelTpl))
      .def("get_tree_root_name", &KDLModel::getTreeRootName,
           DOC(mplib, kdl, KDLModelTpl, getTreeRootName))
      .def("chain_IK_LMA", &KDLModel::chainIKLMA, py::arg("index"), py::arg("q_init"),
           py::arg("goal_pose"), DOC(mplib, kdl, KDLModelTpl, chainIKLMA))
      .def("chain_IK_NR", &KDLModel::chainIKNR, py::arg("index"), py::arg("q_init"),
           py::arg("goal_pose"), DOC(mplib, kdl, KDLModelTpl, chainIKNR))
      .def("chain_IK_NR_JL", &KDLModel::chainIKNRJL, py::arg("index"),
           py::arg("q_init"), py::arg("goal_pose"), py::arg("q_min"), py::arg("q_max"),
           DOC(mplib, kdl, KDLModelTpl, chainIKNRJL))
      .def("tree_IK_NR_JL", &KDLModel::TreeIKNRJL, py::arg("endpoints"),
           py::arg("q_init"), py::arg("goal_poses"), py::arg("q_min"), py::arg("q_max"),
           DOC(mplib, kdl, KDLModelTpl, TreeIKNRJL));
}

}  // namespace mplib
