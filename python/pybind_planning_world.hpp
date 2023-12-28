#pragma once

#include <vector>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include "fcl/narrowphase/collision_request.h"
#include "macros_utils.hpp"
#include "../src/planning_world.h"

namespace py = pybind11;

using DATATYPE = double;

using CollisionObject = fcl::CollisionObject<DATATYPE>;
using CollisionObject_ptr = std::shared_ptr<CollisionObject>;

using PlanningWorld = PlanningWorldTpl<DATATYPE>;
using WorldCollisionResult = WorldCollisionResultTpl<DATATYPE>;
using ArticulatedModel_ptr = ArticulatedModelTpl_ptr<DATATYPE>;

std::string planning_world_doc = R"(
    Planning world for collision checking.
    Args:
        articulations: list of articulated models
        articulation_names: list of names for articulated models
        normal_objects: list of non-articulated collision objects
        normal_object_names: list of names for normal collision objects
        plan_articulation_id: index of the articulated model to be used for planning
    Returns:
        PlanningWorld object)";

std::string get_articulations_doc = R"(
    Get the list of articulated models.
    Returns:
        list of articulated models as pointers)";

std::string get_normal_objects_doc = R"(
    Get the list of non-articulated collision objects.
    Returns:
        list of non-articulated collision objects)";

std::string add_articulation_doc = R"(
    Add an articulated model to the planning world.
    Args:
        model: articulated model to be added
        name: name of the articulated model
    Returns:
        None)";

std::string add_articulations_doc = R"(
    Add a list of articulated models to the planning world.
    Args:
        models: list of articulated models to be added
        names: list of names of the articulated models
    Returns:
        None)";

std::string set_normal_object_doc = R"(
    Add a non-articulated collision object to the planning world.
    Args:
        name: name of the non-articulated collision object
        collision_object: non-articulated collision object to be added
    Returns:
        None)";

std::string remove_normal_object_doc = R"(
    Remove a non-articulated collision object from the planning world.
    Args:
        name: name of the non-articulated collision object
    Returns:
        None)";

std::string set_qpos_single_doc = R"(
    Set the joint qpos of the articulated model.
    Args:
        index: index of the articulated model
        qpos: joint angles of the *movegroup only*
    Returns:
        None)";

std::string set_qpos_all_doc = R"(
    Set the joint qpos of all articulated models.
    Args:
        qpos: joint angles of all the models (*movegroup only*)
    Returns:
        None)";

std::string collide_doc = R"(
    Check collision between all objects.
    Returns:
        True if collision happens)";

std::string self_collide_doc = R"(
    Check collision between the articulated model and itself.
    Args:
        index: index of the articulated model
        request: collision request params. can leave empty for default value
    Returns:
        List of WorldCollisionResult objects)";

std::string collide_with_others_doc = R"(
    Check collision between the articulated model and other objects.
    Args:
        index: index of the articulated model
        request: collision request params. can leave empty for default value
    Returns:
        List of WorldCollisionResult objects)";

std::string collide_full_doc = R"(
    Check collision between the articulated model and all objects.
    Args:
        index: index of the articulated model
        request: collision request params. can leave empty for default value
    Returns:
        List of WorldCollisionResult objects)";

std::string set_use_point_cloud_doc = R"(
    Set whether to use point cloud for collision checking.
    Args:
        use: whether to use point cloud
    Returns:
        None)";

std::string update_point_cloud_doc = R"(
    Update the point cloud for collision checking.
    Args:
        vertices: vertices of the point cloud
        radius: radius of each point in the point cloud
    Returns:
        None)";

std::string set_use_attach_doc = R"(
    Set whether to use attached tool for collision checking.
    Args:
        use: whether to use attached tool
    Returns:
        None)";

std::string remove_attach_doc = R"(
    Remove the attached tool.
    Returns:
        None)";

std::string update_attached_tool_doc = R"(
    Update the attached tool.
    Args:
        p_geom: fcl collision geometry of the attached tool
        link_id: link id of the attached tool
        pose: pose of the attached tool [x, y, z, qw, qx, qy, qz]
    Returns:
        None)";

std::string update_attached_sphere_doc = R"(
    Add sphere as the attached tool.
    Args:
        radius: radius of the sphere
        link_id: link id of the attached sphere
        pose: pose of the attached sphere [x, y, z, qw, qx, qy, qz]
    Returns:
        None)";

std::string update_attached_box_doc = R"(
    Add box as the attached tool.
    Args:
        size: size of the box [x, y, z]
        link_id: link id of the attached box
        pose: pose of the attached box [x, y, z, qw, qx, qy, qz]
    Returns:
        None)";

std::string update_attached_mesh_doc = R"(
    Add mesh as the attached tool.
    Args:
        mesh_path: path to the mesh file
        link_id: link id of the attached mesh
        pose: pose of the attached mesh [x, y, z, qw, qx, qy, qz]
    Returns:
        None)";

std::string print_attached_tool_pose_doc = R"(
    Print the pose of the attached tool.
    Returns:
        None)";

std::string world_collision_result_doc = R"(
    Result of the collision checking.
    Attributes:
        res: whether collision happens
        object_name1: name of the first object
        object_name2: name of the second object
        collision_type: type of the collision
        link_name1: link name of the first object in collision
        link_name2: link name of the second object in collision)";

void build_planning_world(py::module &m_all) {
  auto m = m_all.def_submodule("planning_world");
  auto PyPlanningWorld = py::class_<PlanningWorld, std::shared_ptr<PlanningWorld>>(m, "PlanningWorld");


  PyPlanningWorld.def(py::init<std::vector<ArticulatedModel_ptr> const &,
                      std::vector<std::string> const &,
                      std::vector<CollisionObject_ptr> const &,
                      std::vector<std::string> const &,
                      int const &>(),
                      py::arg("articulations"), py::arg("articulation_names"), 
                      py::arg("normal_objects"), py::arg("normal_object_names"),
                      py::arg("plan_articulation_id") = 0, planning_world_doc.c_str())
    .def("get_articulations", &PlanningWorld::getArticulations, get_articulations_doc.c_str())
    .def("add_articulation", &PlanningWorld::addArticulation, py::arg("model"), py::arg("name"), add_articulation_doc.c_str())
    .def("add_articulations", &PlanningWorld::addArticulations, py::arg("models"), py::arg("names"), add_articulations_doc.c_str())
    .def("get_normal_objects", &PlanningWorld::getNormalObjects, get_normal_objects_doc.c_str())
    .def("set_normal_object", &PlanningWorld::setNormalObject, py::arg("collision_object"), py::arg("name"), set_normal_object_doc.c_str())
    .def("remove_normal_object", &PlanningWorld::removeNormalObject, py::arg("name"), remove_normal_object_doc.c_str())
    .def("set_qpos", &PlanningWorld::setQpos, py::arg("index"), py::arg("qpos"), set_qpos_single_doc.c_str())
    .def("set_qpos_all", &PlanningWorld::setQposAll, py::arg("qpos"), set_qpos_all_doc.c_str())
    .def("collide", &PlanningWorld::collide, collide_doc.c_str())
    .def("self_collide", &PlanningWorld::selfCollide, py::arg("index")=0, py::arg("request")=CollisionRequest(), self_collide_doc.c_str())
    .def("collide_with_others", &PlanningWorld::collideWithOthers, py::arg("index")=0, py::arg("request")=CollisionRequest(), collide_with_others_doc.c_str())
    .def("collide_full", &PlanningWorld::collideFull, py::arg("index")=0, py::arg("request")=CollisionRequest(), collide_full_doc.c_str())
    .def("set_use_point_cloud", &PlanningWorld::setUsePointCloud, py::arg("use"), set_use_point_cloud_doc.c_str())
    .def("update_point_cloud", &PlanningWorld::updatePointCloud, py::arg("vertices"), py::arg("radius"), update_point_cloud_doc.c_str())
    .def("set_use_attach", &PlanningWorld::setUseAttach, py::arg("use"), set_use_attach_doc.c_str())
    .def("remove_attach", &PlanningWorld::removeAttach, remove_attach_doc.c_str())
    .def("update_attached_tool", &PlanningWorld::updateAttachedTool, py::arg("p_geom"), py::arg("link_id"), py::arg("pose"), update_attached_tool_doc.c_str())
    .def("update_attached_sphere", &PlanningWorld::updateAttachedSphere, py::arg("radius"), py::arg("link_id"), py::arg("pose"), update_attached_sphere_doc.c_str())
    .def("update_attached_box", &PlanningWorld::updateAttachedBox, py::arg("size"), py::arg("link_id"), py::arg("pose"))
    .def("update_attached_mesh", &PlanningWorld::updateAttachedMesh, py::arg("mesh_path"), py::arg("link_id"), py::arg("pose"), update_attached_mesh_doc.c_str())
    .def("print_attached_tool_pose", &PlanningWorld::printAttachedToolPose, print_attached_tool_pose_doc.c_str())
    .def_readonly("use_point_cloud", &PlanningWorld::use_point_cloud)
    .def_readonly("use_attach", &PlanningWorld::use_attach);

  auto PyWorldCollisionResult = py::class_<WorldCollisionResult, std::shared_ptr<WorldCollisionResult>>(m, "WorldCollisionResult", world_collision_result_doc.c_str());
  PyWorldCollisionResult.def_readonly("res", &WorldCollisionResult::res)
    .def_readonly("object_name1", &WorldCollisionResult::object_name1)
    .def_readonly("object_name2", &WorldCollisionResult::object_name2)
    .def_readonly("collision_type", &WorldCollisionResult::collision_type)
    //.def_readonly("object_id1", &WorldCollisionResult::object_id1)
    //.def_readonly("object_id2", &WorldCollisionResult::object_id2)
    //.def_readonly("object_type1", &WorldCollisionResult::object_type1)
    //.def_readonly("object_type2", &WorldCollisionResult::object_type2)
    .def_readonly("link_name1", &WorldCollisionResult::link_name1)
    .def_readonly("link_name2", &WorldCollisionResult::link_name2);
}