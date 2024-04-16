#include <memory>

#include <pybind11/pybind11.h>

#include "docstring/collision_detection/collision_common.h"
#include "mplib/collision_detection/collision_common.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib::collision_detection {

using CollisionResult = ::fcl::CollisionResult<S>;
using DistanceResult = ::fcl::DistanceResult<S>;
using WorldCollisionResult = WorldCollisionResultTpl<S>;
using WorldDistanceResult = WorldDistanceResultTpl<S>;

void build_pycollision_common(py::module &m) {
  auto PyWorldCollisionResult =
      py::class_<WorldCollisionResult, std::shared_ptr<WorldCollisionResult>>(
          m, "WorldCollisionResult",
          DOC(mplib, collision_detection, WorldCollisionResultTpl));
  PyWorldCollisionResult
      .def(py::init<>(), DOC(mplib, collision_detection, WorldCollisionResultTpl,
                             WorldCollisionResultTpl))
      .def(py::init<const CollisionResult &, const std::string &, const std::string &,
                    const std::string &, const std::string &, const std::string &>(),
           py::arg("res"), py::arg("collision_type"), py::arg("object_name1"),
           py::arg("object_name2"), py::arg("link_name1"), py::arg("link_name2"),
           DOC(mplib, collision_detection, WorldCollisionResultTpl,
               WorldCollisionResultTpl, 2))
      .def_readonly("res", &WorldCollisionResult::res,
                    DOC(mplib, collision_detection, WorldCollisionResultTpl, res))
      .def_readonly(
          "collision_type", &WorldCollisionResult::collision_type,
          DOC(mplib, collision_detection, WorldCollisionResultTpl, collision_type))
      .def_readonly(
          "object_name1", &WorldCollisionResult::object_name1,
          DOC(mplib, collision_detection, WorldCollisionResultTpl, object_name1))
      .def_readonly(
          "object_name2", &WorldCollisionResult::object_name2,
          DOC(mplib, collision_detection, WorldCollisionResultTpl, object_name2))
      .def_readonly(
          "link_name1", &WorldCollisionResult::link_name1,
          DOC(mplib, collision_detection, WorldCollisionResultTpl, link_name1))
      .def_readonly(
          "link_name2", &WorldCollisionResult::link_name2,
          DOC(mplib, collision_detection, WorldCollisionResultTpl, link_name2));

  auto PyWorldDistanceResult =
      py::class_<WorldDistanceResult, std::shared_ptr<WorldDistanceResult>>(
          m, "WorldDistanceResult",
          DOC(mplib, collision_detection, WorldDistanceResultTpl));
  PyWorldDistanceResult
      .def(py::init<>(), DOC(mplib, collision_detection, WorldDistanceResultTpl,
                             WorldDistanceResultTpl))
      .def(py::init<const DistanceResult &, S, const std::string &, const std::string &,
                    const std::string &, const std::string &, const std::string &>(),
           py::arg("res"), py::arg("min_distance"), py::arg("distance_type"),
           py::arg("object_name1"), py::arg("object_name2"), py::arg("link_name1"),
           py::arg("link_name2"),
           DOC(mplib, collision_detection, WorldDistanceResultTpl,
               WorldDistanceResultTpl, 2))
      .def_readonly("res", &WorldDistanceResult::res,
                    DOC(mplib, collision_detection, WorldDistanceResultTpl, res))
      .def_readonly(
          "min_distance", &WorldDistanceResult::min_distance,
          DOC(mplib, collision_detection, WorldDistanceResultTpl, min_distance))
      .def_readonly(
          "distance_type", &WorldDistanceResult::distance_type,
          DOC(mplib, collision_detection, WorldDistanceResultTpl, distance_type))
      .def_readonly(
          "object_name1", &WorldDistanceResult::object_name1,
          DOC(mplib, collision_detection, WorldDistanceResultTpl, object_name1))
      .def_readonly(
          "object_name2", &WorldDistanceResult::object_name2,
          DOC(mplib, collision_detection, WorldDistanceResultTpl, object_name2))
      .def_readonly("link_name1", &WorldDistanceResult::link_name1,
                    DOC(mplib, collision_detection, WorldDistanceResultTpl, link_name1))
      .def_readonly(
          "link_name2", &WorldDistanceResult::link_name2,
          DOC(mplib, collision_detection, WorldDistanceResultTpl, link_name2));
}

}  // namespace mplib::collision_detection
