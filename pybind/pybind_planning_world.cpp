#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/planning_world.h"
#include "mplib/planning_world.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using PlanningWorld = PlanningWorldTpl<S>;

using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;
using CollisionRequest = fcl::CollisionRequest<S>;
using DistanceRequest = fcl::DistanceRequest<S>;
using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
using FCLObjectPtr = collision_detection::FCLObjectPtr<S>;

void build_pyplanning_world(py::module &pymp) {
  auto PyPlanningWorld = py::class_<PlanningWorld, std::shared_ptr<PlanningWorld>>(
      pymp, "PlanningWorld", DOC(mplib, PlanningWorldTpl));

  PyPlanningWorld
      .def(py::init<const std::vector<ArticulatedModelPtr> &,
                    const std::vector<FCLObjectPtr> &>(),
           py::arg("articulations"), py::arg("objects") = std::vector<FCLObjectPtr>(),
           DOC(mplib, PlanningWorldTpl, PlanningWorldTpl))

      .def("get_articulation_names", &PlanningWorld::getArticulationNames,
           DOC(mplib, PlanningWorldTpl, getArticulationNames))
      .def("get_planned_articulations", &PlanningWorld::getPlannedArticulations,
           DOC(mplib, PlanningWorldTpl, getPlannedArticulations))
      .def("get_articulation", &PlanningWorld::getArticulation, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, getArticulation))
      .def("has_articulation", &PlanningWorld::hasArticulation, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, hasArticulation))
      .def("add_articulation", &PlanningWorld::addArticulation, py::arg("model"),
           py::arg("planned") = false, DOC(mplib, PlanningWorldTpl, addArticulation))
      .def("remove_articulation", &PlanningWorld::removeArticulation, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, removeArticulation))
      .def("is_articulation_planned", &PlanningWorld::isArticulationPlanned,
           py::arg("name"), DOC(mplib, PlanningWorldTpl, isArticulationPlanned))
      .def("set_articulation_planned", &PlanningWorld::setArticulationPlanned,
           py::arg("name"), py::arg("planned"),
           DOC(mplib, PlanningWorldTpl, setArticulationPlanned))

      .def("get_object_names", &PlanningWorld::getObjectNames,
           DOC(mplib, PlanningWorldTpl, getObjectNames))
      .def("get_object", &PlanningWorld::getObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, getObject))
      .def("has_object", &PlanningWorld::hasObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, hasObject))
      .def("add_object",
           py::overload_cast<const FCLObjectPtr &>(&PlanningWorld::addObject),
           py::arg("fcl_obj"), DOC(mplib, PlanningWorldTpl, addObject))
      .def("add_object",
           py::overload_cast<const std::string &, const CollisionObjectPtr &>(
               &PlanningWorld::addObject),
           py::arg("name"), py::arg("collision_object"),
           DOC(mplib, PlanningWorldTpl, addObject, 2))
      .def("add_point_cloud", &PlanningWorld::addPointCloud, py::arg("name"),
           py::arg("vertices"), py::arg("resolution") = 0.01,
           DOC(mplib, PlanningWorldTpl, addPointCloud))
      .def("remove_object", &PlanningWorld::removeObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, removeObject))

      .def("is_object_attached", &PlanningWorld::isObjectAttached, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, isObjectAttached))
      .def("get_attached_object", &PlanningWorld::getAttachedObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, getAttachedObject))
      .def("attach_object",
           py::overload_cast<const std::string &, const std::string &, int,
                             const std::vector<std::string> &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"),
           py::arg("touch_links"), DOC(mplib, PlanningWorldTpl, attachObject))
      .def("attach_object",
           py::overload_cast<const std::string &, const std::string &, int>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"),
           DOC(mplib, PlanningWorldTpl, attachObject, 2))
      .def("attach_object",
           py::overload_cast<const std::string &, const std::string &, int,
                             const Pose<S> &, const std::vector<std::string> &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           py::arg("touch_links"), DOC(mplib, PlanningWorldTpl, attachObject, 3))
      .def("attach_object",
           py::overload_cast<const std::string &, const std::string &, int,
                             const Pose<S> &>(&PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, attachObject, 4))
      .def("attach_object",
           py::overload_cast<const std::string &, const CollisionGeometryPtr &,
                             const std::string &, int, const Pose<S> &,
                             const std::vector<std::string> &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("p_geom"), py::arg("art_name"), py::arg("link_id"),
           py::arg("pose"), py::arg("touch_links"),
           DOC(mplib, PlanningWorldTpl, attachObject, 5))
      .def("attach_object",
           py::overload_cast<const std::string &, const CollisionGeometryPtr &,
                             const std::string &, int, const Pose<S> &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("p_geom"), py::arg("art_name"), py::arg("link_id"),
           py::arg("pose"), DOC(mplib, PlanningWorldTpl, attachObject, 6))
      .def("attach_sphere", &PlanningWorld::attachSphere, py::arg("radius"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, attachSphere))
      .def("attach_box", &PlanningWorld::attachBox, py::arg("size"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, attachBox))
      .def("attach_mesh", &PlanningWorld::attachMesh, py::arg("mesh_path"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"), py::kw_only(),
           py::arg("convex") = false, DOC(mplib, PlanningWorldTpl, attachMesh))
      .def("detach_object", &PlanningWorld::detachObject, py::arg("name"),
           py::arg("also_remove") = false, DOC(mplib, PlanningWorldTpl, detachObject))
      .def("print_attached_body_pose", &PlanningWorld::printAttachedBodyPose,
           DOC(mplib, PlanningWorldTpl, printAttachedBodyPose))

      .def("set_qpos", &PlanningWorld::setQpos, py::arg("name"), py::arg("qpos"),
           DOC(mplib, PlanningWorldTpl, setQpos))
      .def("set_qpos_all", &PlanningWorld::setQposAll, py::arg("state"),
           DOC(mplib, PlanningWorldTpl, setQposAll))

      .def("get_allowed_collision_matrix", &PlanningWorld::getAllowedCollisionMatrix,
           DOC(mplib, PlanningWorldTpl, getAllowedCollisionMatrix))

      .def("is_state_colliding", &PlanningWorld::isStateColliding,
           DOC(mplib, PlanningWorldTpl, isStateColliding))
      .def("check_self_collision", &PlanningWorld::checkSelfCollision,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, checkSelfCollision))
      .def("check_robot_collision", &PlanningWorld::checkRobotCollision,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, checkRobotCollision))
      .def("check_collision", &PlanningWorld::checkCollision,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, checkCollision))

      .def("distance_to_self_collision", &PlanningWorld::distanceToSelfCollision,
           DOC(mplib, PlanningWorldTpl, distanceToSelfCollision))
      .def("distance_self", &PlanningWorld::distanceSelf,
           py::arg("request") = DistanceRequest(),
           DOC(mplib, PlanningWorldTpl, distanceSelf))
      .def("distance_to_robot_collision", &PlanningWorld::distanceToRobotCollision,
           DOC(mplib, PlanningWorldTpl, distanceToRobotCollision))
      .def("distance_robot", &PlanningWorld::distanceRobot,
           py::arg("request") = DistanceRequest(),
           DOC(mplib, PlanningWorldTpl, distanceRobot))
      .def("distance_to_collision", &PlanningWorld::distanceToCollision,
           DOC(mplib, PlanningWorldTpl, distanceToCollision))
      .def("distance", &PlanningWorld::distance, py::arg("request") = DistanceRequest(),
           DOC(mplib, PlanningWorldTpl, distance));
}

}  // namespace mplib
