#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace mplib::collision_detection {

namespace fcl {

void build_pyfcl(py::module &pyfcl);
void build_pyfcl_model(py::module &pyfcl);
void build_pyfcl_utils(py::module &pyfcl);
void build_pyfcl_collision_common(py::module &pyfcl);

}  // namespace fcl

void build_pycollision_common(py::module &m);
void build_pycollision_matrix(py::module &m);

void build_pycollision_detection(py::module &pymp) {
  auto m = pymp.def_submodule("collision_detection", "Collision detection submodule");
  build_pycollision_common(m);
  build_pycollision_matrix(m);

  auto pyfcl = m.def_submodule("fcl", "FCL submodule");
  fcl::build_pyfcl(pyfcl);
  fcl::build_pyfcl_model(pyfcl);
  fcl::build_pyfcl_utils(pyfcl);
  fcl::build_pyfcl_collision_common(pyfcl);
}

}  // namespace mplib::collision_detection
