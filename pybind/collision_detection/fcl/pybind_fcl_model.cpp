#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/collision_detection/fcl/fcl_model.h"
#include "mplib/collision_detection/fcl/fcl_model.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib::collision_detection::fcl {

using FCLModel = FCLModelTpl<S>;
using FCLModelPtr = FCLModelTplPtr<S>;

using CollisionRequest = fcl::CollisionRequest<S>;
using DistanceRequest = fcl::DistanceRequest<S>;

void build_pyfcl_model(py::module &m) {
  auto PyFCLModel = py::class_<FCLModel, std::shared_ptr<FCLModel>>(
      m, "FCLModel", DOC(mplib, collision_detection, fcl, FCLModelTpl));
  PyFCLModel
      .def(py::init<const std::string &, bool, bool>(), py::arg("urdf_filename"),
           py::kw_only(), py::arg("convex") = false, py::arg("verbose") = false,
           DOC(mplib, collision_detection, fcl, FCLModelTpl, FCLModelTpl))

      .def_static(
          "create_from_urdf_string",
          [](const std::string &urdf_string,
             const std::vector<FCLObjectPtr<S>> &collision_links, bool verbose) {
            std::shared_ptr<FCLModel> fcl_model =
                FCLModel::createFromURDFString(urdf_string, collision_links, verbose);
            return fcl_model;
          },
          py::arg("urdf_string"), py::arg("collision_links"), py::kw_only(),
          py::arg("verbose") = false,
          DOC(mplib, collision_detection, fcl, FCLModelTpl, createFromURDFString))

      .def_property_readonly("name", &FCLModel::getName,
                             DOC(mplib, collision_detection, fcl, FCLModelTpl, name))
      .def("get_name", &FCLModel::getName,
           DOC(mplib, collision_detection, fcl, FCLModelTpl, getName))

      .def("get_collision_objects", &FCLModel::getCollisionObjects,
           DOC(mplib, collision_detection, fcl, FCLModelTpl, getCollisionObjects))
      .def("get_collision_link_names", &FCLModel::getCollisionLinkNames,
           DOC(mplib, collision_detection, fcl, FCLModelTpl, getCollisionLinkNames))
      .def("get_collision_pairs", &FCLModel::getCollisionPairs,
           DOC(mplib, collision_detection, fcl, FCLModelTpl, getCollisionPairs))

      .def("set_link_order", &FCLModel::setLinkOrder, py::arg("names"),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, setLinkOrder))

      .def("remove_collision_pairs_from_srdf", &FCLModel::removeCollisionPairsFromSRDF,
           py::arg("srdf_filename"),
           DOC(mplib, collision_detection, fcl, FCLModelTpl,
               removeCollisionPairsFromSRDF))

      .def("update_collision_objects", &FCLModel::updateCollisionObjects,
           py::arg("link_poses"),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, updateCollisionObjects))

      .def("is_state_colliding", &FCLModel::isStateColliding, py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, isStateColliding))
      .def("check_self_collision", &FCLModel::checkSelfCollision,
           py::arg("request") = CollisionRequest(), py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, checkSelfCollision))
      .def("check_collision_with",
           py::overload_cast<const FCLModelPtr &, const CollisionRequest &,
                             const AllowedCollisionMatrixPtr &>(
               &FCLModel::checkCollisionWith, py::const_),
           py::arg("other"), py::arg("request") = CollisionRequest(), py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, checkCollisionWith))
      .def("check_collision_with",
           py::overload_cast<const FCLObjectPtr<S> &, const CollisionRequest &,
                             const AllowedCollisionMatrixPtr &>(
               &FCLModel::checkCollisionWith, py::const_),
           py::arg("object"), py::arg("request") = CollisionRequest(), py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, checkCollisionWith, 2))

      .def("distance_to_self_collision", &FCLModel::distanceToSelfCollision,
           py::kw_only(), py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, distanceToSelfCollision))
      .def("distance_self", &FCLModel::distanceSelf,
           py::arg("request") = DistanceRequest(), py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, distanceSelf))
      .def("distance_to_collision_with",
           py::overload_cast<const FCLModelPtr &, const AllowedCollisionMatrixPtr &>(
               &FCLModel::distanceToCollisionWith, py::const_),
           py::arg("other"), py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, distanceToCollisionWith))
      .def("distance_with",
           py::overload_cast<const FCLModelPtr &, const DistanceRequest &,
                             const AllowedCollisionMatrixPtr &>(&FCLModel::distanceWith,
                                                                py::const_),
           py::arg("other"), py::arg("request") = DistanceRequest(), py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, distanceWith))
      .def(
          "distance_to_collision_with",
          py::overload_cast<const FCLObjectPtr<S> &, const AllowedCollisionMatrixPtr &>(
              &FCLModel::distanceToCollisionWith, py::const_),
          py::arg("object"), py::kw_only(),
          py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
          DOC(mplib, collision_detection, fcl, FCLModelTpl, distanceToCollisionWith, 2))
      .def("distance_with",
           py::overload_cast<const FCLObjectPtr<S> &, const DistanceRequest &,
                             const AllowedCollisionMatrixPtr &>(&FCLModel::distanceWith,
                                                                py::const_),
           py::arg("object"), py::arg("request") = DistanceRequest(), py::kw_only(),
           py::arg("acm") = std::make_shared<AllowedCollisionMatrix>(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, distanceWith, 2));
}

}  // namespace mplib::collision_detection::fcl
