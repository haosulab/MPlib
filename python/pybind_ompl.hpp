#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include "../src/ompl_planner.h"

namespace py = pybind11;

using DATATYPE = double;

using OMPLPlanner = OMPLPlannerTpl<DATATYPE>;
using PlannerStatus = ob::PlannerStatus;
using Path = ob::Path;
using PathGeometric = og::PathGeometric;


template<typename T>
py::array_t<T> make_array(std::vector<T> const &values) {
    return py::array_t<T>(values.size(), values.data());
}

void build_pyompl(py::module &m_all) {
    auto m = m_all.def_submodule("ompl");

    auto PyOMPLPlanner = py::class_<OMPLPlanner, std::shared_ptr<OMPLPlanner>>(m, "OMPLPlanner");
    PyOMPLPlanner.def(py::init<PlanningWorldTpl_ptr<DATATYPE> const &, int, bool>(),
                      py::arg("world"),
                      py::arg("robot_idx")=0)
                 .def("plan", &OMPLPlanner::plan,
                      py::arg("start_state"),
                      py::arg("goal_states"),
                      py::arg("planner_name")="RRTConnect",
                      py::arg("time")=1.0,
                      py::arg("range")=0.0,
                      py::arg("verbose")=false);
}
