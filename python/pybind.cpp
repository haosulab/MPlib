#include <pybind11/pybind11.h>

#include "pybind_articulation.hpp"
#include "pybind_fcl.hpp"
#include "pybind_kdl.hpp"
#include "pybind_ompl.hpp"
#include "pybind_pinocchio.hpp"
#include "pybind_planning_world.hpp"

namespace py = pybind11;

namespace mplib {

PYBIND11_MODULE(pymp, m) {
  m.doc() = "Motion planning python binding";
  build_pyfcl(m);
  build_pypinocchio(m);
  build_pykdl(m);
  build_pyarticulation(m);
  build_planning_world(m);
  build_pyompl(m);
  // build_pytopp(m);
}

}  // namespace mplib
