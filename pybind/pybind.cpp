#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace mplib {

namespace collision_detection {

void build_pycollision_detection(py::module &pymp);

}  // namespace collision_detection

namespace kinematics {

void build_pykinematics(py::module &pymp);

}  // namespace kinematics

namespace planning {

void build_pyplanning(py::module &pymp);

}  // namespace planning

void build_pyarticulated_model(py::module &pymp);
void build_pyplanning_world(py::module &pymp);

PYBIND11_MODULE(pymp, m) {
  m.doc() = "Motion planning python binding";

  collision_detection::build_pycollision_detection(m);
  kinematics::build_pykinematics(m);
  planning::build_pyplanning(m);

  build_pyarticulated_model(m);
  build_pyplanning_world(m);
}

}  // namespace mplib
