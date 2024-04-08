#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/collision_detection/fcl/collision_common.h"
#include "mplib/collision_detection/fcl/collision_common.h"
#include "mplib/types.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib::collision_detection::fcl {

using CollisionRequest = fcl::CollisionRequest<S>;
using CollisionResult = fcl::CollisionResult<S>;
using DistanceRequest = fcl::DistanceRequest<S>;
using DistanceResult = fcl::DistanceResult<S>;

void build_pyfcl_collision_common(py::module &m) {
  auto PyFCLObject = py::class_<FCLObject<S>, std::shared_ptr<FCLObject<S>>>(
      m, "FCLObject", DOC(mplib, collision_detection, fcl, FCLObject));

  // TODO(merge): mplib.Pose
  PyFCLObject
      .def(py::init<const std::string &>(), py::arg("name"),
           DOC(mplib, collision_detection, fcl, FCLObject, FCLObject))
      .def(py::init<const std::string &, const Isometry3<S> &,
                    const std::vector<fcl::CollisionObjectPtr<S>> &,
                    const std::vector<Isometry3<S>> &>(),
           py::arg("name"), py::arg("pose"), py::arg("shapes"), py::arg("shape_poses"),
           DOC(mplib, collision_detection, fcl, FCLObject, FCLObject, 2))
      .def_readonly("name", &FCLObject<S>::name,
                    DOC(mplib, collision_detection, fcl, FCLObject, name))
      .def_readonly("pose", &FCLObject<S>::pose,
                    DOC(mplib, collision_detection, fcl, FCLObject, pose))
      .def_readonly("shapes", &FCLObject<S>::shapes,
                    DOC(mplib, collision_detection, fcl, FCLObject, shapes))
      .def_readonly("shape_poses", &FCLObject<S>::shape_poses,
                    DOC(mplib, collision_detection, fcl, FCLObject, shape_poses));

  // collide / distance functions
  m.def(
       "collide",
       [](const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,
          const CollisionRequest &request) {
         CollisionResult result;
         collide(obj1, obj2, request, result);
         return result;
       },
       py::arg("obj1"), py::arg("obj2"), py::arg("request") = CollisionRequest())
      .def(
          "distance",
          [](const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,
             const DistanceRequest &request) {
            DistanceResult result;
            distance(obj1, obj2, request, result);
            return result;
          },
          py::arg("obj1"), py::arg("obj2"), py::arg("request") = DistanceRequest());
}

}  // namespace mplib::collision_detection::fcl
