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

#ifdef USE_SINGLE
using DATATYPE = float;
#else
using DATATYPE = double;
#endif

using CollisionObject = fcl::CollisionObject<DATATYPE>;
using CollisionObject_ptr = std::shared_ptr<CollisionObject>;

using PlanningWorld = PlanningWorldTpl<DATATYPE>;
using WorldCollisionResult = WorldCollisionResultTpl<DATATYPE>;
using ArticulatedModel_ptr = ArticulatedModelTpl_ptr<DATATYPE>;


void build_planning_world(py::module &m_all) {
    auto m = m_all.def_submodule("planning_world");
    auto PyPlanningWorld = py::class_<PlanningWorld, std::shared_ptr<PlanningWorld>>(m, "PlanningWorld");


    PyPlanningWorld.def(py::init<std::vector<ArticulatedModel_ptr> const &,
                                std::vector<std::string> const &,
                                std::vector<CollisionObject_ptr> const &,
                                std::vector<std::string> const &,
                                int const &>(),
                                //std::vector<bool> const &>(),
                        py::arg("articulations"), py::arg("articulation_names"), 
                        py::arg("normal_objects"), py::arg("normal_object_names"), py::arg("plan_articulation_id") = 0)
            .def("get_articulations", &PlanningWorld::getArticulations)
            //.def("get_articulation_flags", &PlanningWorld::getArticulationFlags)
            .def("get_normal_objects", &PlanningWorld::getNormalObjects)
            .def("add_articulation", &PlanningWorld::addArticulation, py::arg("model"), py::arg("name"))
            .def("add_articulations", &PlanningWorld::addArticulations, py::arg("models"), py::arg("names"))
            .def("add_normal_object", &PlanningWorld::addNormalObject, py::arg("collision_object"), py::arg("name"))
            .def("add_normal_objects", &PlanningWorld::addNormalObjects, py::arg("collision_objects"), py::arg("names"))
            .def("set_qpos", &PlanningWorld::setQpos, py::arg("index"),
                 py::arg("qpos"))
            .def("set_qpos_all", &PlanningWorld::setQposAll, py::arg("qpos"))
            .def("collide", &PlanningWorld::collide)
            .def("self_collide", &PlanningWorld::selfCollide, py::arg("index")=0, py::arg("request")=CollisionRequest())
            .def("collide_with_others", &PlanningWorld::collideWithOthers, py::arg("index")=0, py::arg("request")=CollisionRequest())
            .def("collide_full", &PlanningWorld::collideFull, py::arg("index")=0, py::arg("request")=CollisionRequest())
            .def("set_use_point_cloud", &PlanningWorld::setUsePointCloud, py::arg("use") = false)
            .def("update_point_cloud", &PlanningWorld::updatePointCloud, py::arg("vertices"), py::arg("resolution") = 0.01)
            .def("set_use_attach", &PlanningWorld::setUseAttach, py::arg("use") = false)
            .def("update_attached_box", &PlanningWorld::updateAttachedBox, py::arg("size"), py::arg("link_id"), py::arg("pose"))
            .def("print_attached_box_pose", &PlanningWorld::printAttachedBoxPose);

    auto PyWorldCollisionResult = py::class_<WorldCollisionResult, std::shared_ptr<WorldCollisionResult>>(m, "WorldCollisionResult");
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