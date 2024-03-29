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
using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;

void build_pyplanning_world(py::module &pymp) {
  auto PyPlanningWorld = py::class_<PlanningWorld, std::shared_ptr<PlanningWorld>>(
      pymp, "PlanningWorld", DOC(mplib, PlanningWorldTpl));

  PyPlanningWorld
      .def(py::init<const std::vector<ArticulatedModelPtr> &,
                    const std::vector<std::string> &,
                    const std::vector<CollisionObjectPtr> &,
                    const std::vector<std::string> &>(),
           py::arg("articulations"), py::arg("articulation_names"),
           py::arg("normal_objects") = std::vector<CollisionObjectPtr>(),
           py::arg("normal_object_names") = std::vector<std::string>(),
           DOC(mplib, PlanningWorldTpl, PlanningWorldTpl))

      .def("get_articulation_names", &PlanningWorld::getArticulationNames,
           DOC(mplib, PlanningWorldTpl, getArticulationNames))
      .def("get_planned_articulations", &PlanningWorld::getPlannedArticulations,
           DOC(mplib, PlanningWorldTpl, getPlannedArticulations))
      .def("get_articulation", &PlanningWorld::getArticulation, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, getArticulation))
      .def("has_articulation", &PlanningWorld::hasArticulation, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, hasArticulation))
      .def("add_articulation", &PlanningWorld::addArticulation, py::arg("name"),
           py::arg("model"), py::arg("planned") = false,
           DOC(mplib, PlanningWorldTpl, addArticulation))
      .def("remove_articulation", &PlanningWorld::removeArticulation, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, removeArticulation))
      .def("is_articulation_planned", &PlanningWorld::isArticulationPlanned,
           py::arg("name"), DOC(mplib, PlanningWorldTpl, isArticulationPlanned))
      .def("set_articulation_planned", &PlanningWorld::setArticulationPlanned,
           py::arg("name"), py::arg("planned"),
           DOC(mplib, PlanningWorldTpl, setArticulationPlanned))

      .def("get_normal_object_names", &PlanningWorld::getNormalObjectNames,
           DOC(mplib, PlanningWorldTpl, getNormalObjectNames))
      .def("get_normal_object", &PlanningWorld::getNormalObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, getNormalObject))
      .def("has_normal_object", &PlanningWorld::hasNormalObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, hasNormalObject))
      .def("add_normal_object", &PlanningWorld::addNormalObject, py::arg("name"),
           py::arg("collision_object"), DOC(mplib, PlanningWorldTpl, addNormalObject))
      .def("add_point_cloud", &PlanningWorld::addPointCloud, py::arg("name"),
           py::arg("vertices"), py::arg("resolution") = 0.01,
           DOC(mplib, PlanningWorldTpl, addPointCloud))
      .def("remove_normal_object", &PlanningWorld::removeNormalObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, removeNormalObject))

      .def("is_normal_object_attached", &PlanningWorld::isNormalObjectAttached,
           py::arg("name"), DOC(mplib, PlanningWorldTpl, isNormalObjectAttached))
      .def("get_attached_object", &PlanningWorld::getAttachedObject, py::arg("name"),
           DOC(mplib, PlanningWorldTpl, getAttachedObject))
      .def("attach_object",
           py::overload_cast<const std::string &, const std::string &, int,
                             const Vector7<S> &, const std::vector<std::string> &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           py::arg("touch_links"), DOC(mplib, PlanningWorldTpl, attachObject))
      .def("attach_object",
           py::overload_cast<const std::string &, const std::string &, int,
                             const Vector7<S> &>(&PlanningWorld::attachObject),
           py::arg("name"), py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, attachObject, 2))
      .def("attach_object",
           py::overload_cast<const std::string &, const CollisionGeometryPtr &,
                             const std::string &, int, const Vector7<S> &,
                             const std::vector<std::string> &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("p_geom"), py::arg("art_name"), py::arg("link_id"),
           py::arg("pose"), py::arg("touch_links"),
           DOC(mplib, PlanningWorldTpl, attachObject, 3))
      .def("attach_object",
           py::overload_cast<const std::string &, const CollisionGeometryPtr &,
                             const std::string &, int, const Vector7<S> &>(
               &PlanningWorld::attachObject),
           py::arg("name"), py::arg("p_geom"), py::arg("art_name"), py::arg("link_id"),
           py::arg("pose"), DOC(mplib, PlanningWorldTpl, attachObject, 4))
      .def("attach_sphere", &PlanningWorld::attachSphere, py::arg("radius"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, attachSphere))
      .def("attach_box", &PlanningWorld::attachBox, py::arg("size"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, attachBox))
      .def("attach_mesh", &PlanningWorld::attachMesh, py::arg("mesh_path"),
           py::arg("art_name"), py::arg("link_id"), py::arg("pose"),
           DOC(mplib, PlanningWorldTpl, attachMesh))
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

      .def("collide", &PlanningWorld::collide, py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, collide))
      .def("self_collide", &PlanningWorld::selfCollide,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, selfCollide))
      .def("collide_with_others", &PlanningWorld::collideWithOthers,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, collideWithOthers))
      .def("collide_full", &PlanningWorld::collideFull,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, PlanningWorldTpl, collideFull));
}

}  // namespace mplib
