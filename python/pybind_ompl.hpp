#pragma once

#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/ompl_planner.h"
#include "mplib/ompl_planner.h"

namespace py = pybind11;

using DATATYPE = double;

using OMPLPlanner = OMPLPlannerTpl<DATATYPE>;
using PlannerStatus = ob::PlannerStatus;
using Path = ob::Path;
using PathGeometric = og::PathGeometric;

template <typename T>
py::array_t<T> make_array(const std::vector<T> &values) {
  return py::array_t<T>(values.size(), values.data());
}

void build_pyompl(py::module &m_all) {
  auto m = m_all.def_submodule("ompl");

  auto PyOMPLPlanner = py::class_<OMPLPlanner, std::shared_ptr<OMPLPlanner>>(
      m, "OMPLPlanner", DOC(OMPLPlannerTpl));

  PyOMPLPlanner
      .def(py::init<const PlanningWorldTpl_ptr<DATATYPE> &, int>(), py::arg("world"),
           py::arg("robot_idx") = 0, DOC(OMPLPlannerTpl, OMPLPlannerTpl))
      .def("plan", &OMPLPlanner::plan, py::arg("start_state"), py::arg("goal_states"),
           py::arg("planner_name") = "RRTConnect", py::arg("time") = 1.0,
           py::arg("range") = 0.0, py::arg("verbose") = false,
           py::arg("fixed_joints") = FixedJoints(),
           py::arg("no_simplification") = false,
           py::arg("constraint_function") = nullptr,
           py::arg("constraint_jacobian") = nullptr,
           py::arg("constraint_tolerance") = 1e-3, DOC(OMPLPlannerTpl, plan))
      .def("simplify_path", &OMPLPlanner::simplify_path, py::arg("path"),
           DOC(OMPLPlannerTpl, simplify_path));

  auto PyFixedJoint = py::class_<FixedJoint, std::shared_ptr<FixedJoint>>(
      m, "FixedJoint", DOC(FixedJoint));
  PyFixedJoint.def(py::init<size_t, size_t, double>(), py::arg("articulation_idx"),
                   py::arg("joint_idx"), py::arg("value"), DOC(FixedJoint, FixedJoint));
  PyFixedJoint.def_readwrite("articulation_idx", &FixedJoint::articulation_idx);
  PyFixedJoint.def_readwrite("joint_idx", &FixedJoint::joint_idx);
  PyFixedJoint.def_readwrite("value", &FixedJoint::value);
}
