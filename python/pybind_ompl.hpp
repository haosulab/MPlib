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

std::string plan_doc = R"(
    Plan a path from start state to goal states.
    Args:
        start_state: start state of the movegroup joints
        goal_states: list of goal states. planner will stop when one of them is reached
        planner_name: name of the planner pick between {RRTConnect, RRT*}
        time: planning time limit
        range: planning range (for RRT family of planners and represents the maximum step size)
        verbose: print debug information
        align_axis: axis to align the end effector z-axis to
        align_angle: angle between the end effector z-axis and the align_axis
        no_simplification: if true, the path will not be simplified
    Returns:
        pair of planner status and path. If planner succeeds, status is "Exact solution.")";

std::string ompl_ctor_doc = R"(
    Args:
        world: planning world
    Returns:
        OMPLPlanner object)";

std::string simplify_path_doc = R"(
    Args:
        path: path to be simplified (numpy array of shape (n, dim))
    Returns:
        simplified path)";

template<typename T>
py::array_t<T> make_array(std::vector<T> const &values) {
    return py::array_t<T>(values.size(), values.data());
}

void build_pyompl(py::module &m_all) {
    auto m = m_all.def_submodule("ompl");

    auto PyOMPLPlanner = py::class_<OMPLPlanner, std::shared_ptr<OMPLPlanner>>(m, "OMPLPlanner");
    PyOMPLPlanner.def(py::init<PlanningWorldTpl_ptr<DATATYPE> const &, int>(),
                      py::arg("world"),
                      py::arg("robot_idx")=0,
                      ompl_ctor_doc.c_str())
                 .def("plan", &OMPLPlanner::plan,
                      py::arg("start_state"),
                      py::arg("goal_states"),
                      py::arg("planner_name")="RRTConnect",
                      py::arg("time")=1.0,
                      py::arg("range")=0.0,
                      py::arg("verbose")=false,
                      py::arg("align_axis")=Eigen::Vector3d(0,0,0),
                      py::arg("align_angle")=0.0,
                      py::arg("no_simplification")=false,
                      plan_doc.c_str())
                 .def("simplify_path", &OMPLPlanner::simplify_path, py::arg("path"), simplify_path_doc.c_str());
}
