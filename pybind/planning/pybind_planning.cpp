#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace mplib::planning {

namespace ompl {

void build_pyompl_planner(py::module &pyompl);
void build_pyfixed_joint(py::module &pyompl);

}  // namespace ompl

void build_pyplanning(py::module &pymp) {
  auto m = pymp.def_submodule("planning", "Planning submodule");

  auto pyompl = m.def_submodule("ompl", "OMPL submodule");
  ompl::build_pyompl_planner(pyompl);
  ompl::build_pyfixed_joint(pyompl);
}

}  // namespace mplib::planning
