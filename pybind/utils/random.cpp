#include "mplib/utils/random.h"

#include <pybind11/pybind11.h>

#include "docstring/utils/random.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

void build_utils_random(py::module &pymp) {
  pymp.def("set_global_seed", &setGlobalSeed<S>, py::arg("seed"),
           DOC(mplib, setGlobalSeed));
}

}  // namespace mplib
