#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "docstring/collision_detection/fcl/fcl_utils.h"
#include "mplib/collision_detection/fcl/fcl_utils.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib::collision_detection::fcl {

void build_pyfcl_utils(py::module &m) {
  m.def("load_mesh_as_BVH", loadMeshAsBVH<S>, py::arg("mesh_path"), py::arg("scale"),
        DOC(mplib, collision_detection, fcl, loadMeshAsBVH));
  m.def("load_mesh_as_Convex", loadMeshAsConvex<S>, py::arg("mesh_path"),
        py::arg("scale"), DOC(mplib, collision_detection, fcl, loadMeshAsConvex));
}

}  // namespace mplib::collision_detection::fcl
