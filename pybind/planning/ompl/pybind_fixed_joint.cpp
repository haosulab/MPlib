#include <memory>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include "docstring/planning/ompl/fixed_joint.h"
#include "mplib/planning/ompl/fixed_joint.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib::planning::ompl {

using FixedJoint = ompl::FixedJointTpl<S>;

void build_pyfixed_joint(py::module &m) {
  auto PyFixedJoint = py::class_<FixedJoint, std::shared_ptr<FixedJoint>>(
      m, "FixedJoint", DOC(mplib, planning, ompl, FixedJointTpl));
  PyFixedJoint.def(py::init<size_t, size_t, S>(), py::arg("articulation_idx"),
                   py::arg("joint_idx"), py::arg("value"),
                   DOC(mplib, planning, ompl, FixedJointTpl, FixedJointTpl));
  PyFixedJoint.def(py::self == py::self,
                   DOC(mplib, planning, ompl, FixedJointTpl, operator_eq));
  PyFixedJoint.def(py::self < py::self,
                   DOC(mplib, planning, ompl, FixedJointTpl, operator_lt));
  PyFixedJoint.def("__hash__", [](const FixedJoint &self) {
    return py::hash(py::make_tuple(self.articulation_idx, self.joint_idx));
  });
  PyFixedJoint.def_readonly("articulation_idx", &FixedJoint::articulation_idx);
  PyFixedJoint.def_readonly("joint_idx", &FixedJoint::joint_idx);
  PyFixedJoint.def_readonly("value", &FixedJoint::value);
}

}  // namespace mplib::planning::ompl
