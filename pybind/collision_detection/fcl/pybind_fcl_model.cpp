#include <memory>
#include <string>
#include <utility>
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

using CollisionRequest = fcl::CollisionRequest<S>;

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
             const std::vector<std::pair<std::string, FCLObjectPtr<S>>>
                 &collision_links,
             bool verbose) {
            std::shared_ptr<FCLModel> fcl_model =
                FCLModel::createFromURDFString(urdf_string, collision_links, verbose);
            return fcl_model;
          },
          py::arg("urdf_string"), py::arg("collision_links"), py::kw_only(),
          py::arg("verbose") = false,
          DOC(mplib, collision_detection, fcl, FCLModelTpl, createFromURDFString))

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

      .def("collide", &FCLModel::collide, py::arg("request") = CollisionRequest(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, collide))
      .def("collide_full", &FCLModel::collideFull,
           py::arg("request") = CollisionRequest(),
           DOC(mplib, collision_detection, fcl, FCLModelTpl, collideFull));
}

}  // namespace mplib::collision_detection::fcl
