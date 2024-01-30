#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace mplib::kinematics {

namespace kdl {

void build_pykdl_model(py::module &pykdl);

}  // namespace kdl

namespace pinocchio {

void build_pypinocchio_model(py::module &pypinocchio);

}  // namespace pinocchio

void build_pykinematics(py::module &pymp) {
  auto m = pymp.def_submodule("kinematics", "Kinematics submodule");

  auto pykdl = m.def_submodule("kdl", "KDL submodule");
  kdl::build_pykdl_model(pykdl);

  auto pypinocchio = m.def_submodule("pinocchio", "Pinocchio submodule");
  pinocchio::build_pypinocchio_model(pypinocchio);
}

}  // namespace mplib::kinematics
