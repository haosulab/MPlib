#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/planning_world.h"
#include "mplib/planning_world.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using PlanningWorld = PlanningWorldTpl<S>;
using WorldCollisionResult = WorldCollisionResultTpl<S>;

using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;
using CollisionRequest = fcl::CollisionRequest<S>;
using DistanceRequest = fcl::DistanceRequest<S>;
using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;

inline void build_planning_world(py::module &m_all) {
  auto m = m_all.def_submodule("planning_world");
  auto PyPlanningWorld = py::class_<PlanningWorld, std::shared_ptr<PlanningWorld>>(
      m, "PlanningWorld", DOC(mplib, PlanningWorldTpl));

  PyPlanningWorld
      .def(py::init<const std::vector<ArticulatedModelPtr> &,
                    const std::vector<std::string> &,
                    const std::vector<CollisionObjectPtr> &,
                    const std::vector<std::string> &, const int &>(),
           py::arg("articulations"), py::arg("articulation_names"),
           py::arg("normal_objects"), py::arg("normal_object_names"),
           py::arg("plan_articulation_id") = 0,
           DOC(mplib, PlanningWorldTpl, PlanningWorldTpl))
      .def("get_articulations", &PlanningWorld::getArticulations,
           DOC(mplib, PlanningWorldTpl, getArticulations))
      .def("add_articulation", &PlanningWorld::addArticulation, py::arg("model"),
           py::arg("name"), DOC(mplib, PlanningWorldTpl, addArticulation))
      .def("add_articulations", &PlanningWorld::addArticulations, py::arg("models"),
           py::arg("names"), DOC(mplib, PlanningWorldTpl, addArticulations))
      .def("get_normal_objects", &PlanningWorld::getNormalObjects,
           DOC(mplib, PlanningWorldTpl, getNormalObjects))
      .def("set_normal_object", &PlanningWorld::setNormalObject,
           py::arg("collision_object"), py::arg("name"),
           DOC(mplib, PlanningWorldTpl, setNormalObject))
      .def("remove_normal_object", &PlanningWorld::removeNormalObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, removeNormalObject))
      .def("set_qpos", &PlanningWorld::setQpos, py::arg("index"), py::arg("qpos"),
           DOC(mplib, PlanningWorldTpl, setQpos))
      .def("set_qpos_all", &PlanningWorld::setQposAll, py::arg("qpos"),
           DOC(mplib, PlanningWorldTpl, setQposAll))
      .def("collide", &PlanningWorld::collide, DOC(mplib, PlanningWorldTpl, collide))
      .def("self_collide", &PlanningWorld::selfCollide, py::arg("index") = 0,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, selfCollide))
      .def("collide_with_others", &PlanningWorld::collideWithOthers,
           py::arg("index") = 0, py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, collideWithOthers))
      .def("collide_full", &PlanningWorld::collideFull, py::arg("index") = 0,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, collideFull))
      .def("set_use_point_cloud", &PlanningWorld::setUsePointCloud, py::arg("use"),
           DOC(mplib, PlanningWorldTpl, setUsePointCloud))
      .def("update_point_cloud", &PlanningWorld::updatePointCloud, py::arg("vertices"),
           py::arg("radius"), DOC(mplib, PlanningWorldTpl, updatePointCloud))
      .def("set_use_attach", &PlanningWorld::setUseAttach, py::arg("use"),
           DOC(mplib, PlanningWorldTpl, setUseAttach))
      .def("remove_attach", &PlanningWorld::removeAttach,
           DOC(mplib, PlanningWorldTpl, removeAttach))
      .def("update_attached_tool", &PlanningWorld::updateAttachedTool,
           py::arg("p_geom"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, updateAttachedTool))
      .def("update_attached_sphere", &PlanningWorld::updateAttachedSphere,
           py::arg("radius"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, updateAttachedSphere))
      .def("update_attached_box", &PlanningWorld::updateAttachedBox, py::arg("size"),
           py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, updateAttachedBox))
      .def("update_attached_mesh", &PlanningWorld::updateAttachedMesh,
           py::arg("mesh_path"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, updateAttachedMesh))
      .def("print_attached_tool_pose", &PlanningWorld::printAttachedToolPose,
           DOC(mplib, PlanningWorldTpl, printAttachedToolPose))
      .def_readonly("use_point_cloud", &PlanningWorld::use_point_cloud_)
      .def_readonly("use_attach", &PlanningWorld::use_attach_);

  auto PyWorldCollisionResult =
      py::class_<WorldCollisionResult, std::shared_ptr<WorldCollisionResult>>(
          m, "WorldCollisionResult", DOC(mplib, WorldCollisionResultTpl));
  PyWorldCollisionResult
      .def_readonly("res", &WorldCollisionResult::res,
                    DOC(mplib, WorldCollisionResultTpl, res))
      .def_readonly("collision_type", &WorldCollisionResult::collision_type,
                    DOC(mplib, WorldCollisionResultTpl, collision_type))
      .def_readonly("object_name1", &WorldCollisionResult::object_name1,
                    DOC(mplib, WorldCollisionResultTpl, object_name1))
      .def_readonly("object_name2", &WorldCollisionResult::object_name2,
                    DOC(mplib, WorldCollisionResultTpl, object_name2))
      .def_readonly("link_name1", &WorldCollisionResult::link_name1,
                    DOC(mplib, WorldCollisionResultTpl, link_name1))
      .def_readonly("link_name2", &WorldCollisionResult::link_name2,
                    DOC(mplib, WorldCollisionResultTpl, link_name2));
}

}  // namespace mplib
