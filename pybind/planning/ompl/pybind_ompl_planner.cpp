#include <memory>

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/planning/ompl/ompl_planner.h"
#include "mplib/planning/ompl/ompl_planner.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib::planning::ompl {

using OMPLPlanner = OMPLPlannerTpl<S>;

using FixedJoints = FixedJointsTpl<S>;

void build_pyompl_planner(py::module &m) {
  auto PyOMPLPlanner = py::class_<OMPLPlanner, std::shared_ptr<OMPLPlanner>>(
      m, "OMPLPlanner", DOC(mplib, planning, ompl, OMPLPlannerTpl));
  PyOMPLPlanner
      .def(py::init<const PlanningWorldTplPtr<S> &>(), py::arg("world"),
           DOC(mplib, planning, ompl, OMPLPlannerTpl, OMPLPlannerTpl))

      .def("plan", &OMPLPlanner::plan, py::arg("start_state"), py::arg("goal_states"),
           py::kw_only(), py::arg("time") = 1.0, py::arg("range") = 0.0,
           py::arg("fixed_joints") = FixedJoints(), py::arg("simplify") = true,
           py::arg("constraint_function") = nullptr,
           py::arg("constraint_jacobian") = nullptr,
           py::arg("constraint_tolerance") = 1e-3, py::arg("verbose") = false,
           DOC(mplib, planning, ompl, OMPLPlannerTpl, plan))

      .def("simplify_path", &OMPLPlanner::simplifyPath, py::arg("path"),
           DOC(mplib, planning, ompl, OMPLPlannerTpl, simplifyPath));
}

}  // namespace mplib::planning::ompl
