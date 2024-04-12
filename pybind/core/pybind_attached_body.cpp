#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/core/attached_body.h"
#include "mplib/core/attached_body.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using AttachedBody = AttachedBodyTpl<S>;
using FCLObjectPtr = collision_detection::FCLObjectPtr<S>;
using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;

void build_pyattached_body(py::module &pymp) {
  auto PyAttachedBody = py::class_<AttachedBody, std::shared_ptr<AttachedBody>>(
      pymp, "AttachedBody", DOC(mplib, AttachedBodyTpl));
  PyAttachedBody
      .def(py::init<const std::string &, const FCLObjectPtr &,
                    const ArticulatedModelPtr &, int, const Pose<S> &,
                    const std::vector<std::string> &>(),
           py::arg("name"), py::arg("object"), py::arg("attached_articulation"),
           py::arg("attached_link_id"), py::arg("pose"),
           py::arg("touch_links") = std::vector<std::string>(),
           DOC(mplib, AttachedBodyTpl, AttachedBodyTpl))
      .def("get_name", &AttachedBody::getName, DOC(mplib, AttachedBodyTpl, getName))
      .def("get_object", &AttachedBody::getObject,
           DOC(mplib, AttachedBodyTpl, getObject))
      .def("get_attached_articulation", &AttachedBody::getAttachedArticulation,
           DOC(mplib, AttachedBodyTpl, getAttachedArticulation))
      .def("get_attached_link_id", &AttachedBody::getAttachedLinkId,
           DOC(mplib, AttachedBodyTpl, getAttachedLinkId))

      .def_property(
          "pose", [](const AttachedBody &body) { return Pose<S>(body.getPose()); },
          [](AttachedBody &body, const Pose<S> &pose) {
            body.setPose(pose.toIsometry());
          },
          DOC(mplib, AttachedBodyTpl, pose))
      .def(
          "get_pose", [](const AttachedBody &body) { return Pose<S>(body.getPose()); },
          DOC(mplib, AttachedBodyTpl, getPose))
      .def(
          "set_pose",
          [](AttachedBody &body, const Pose<S> &pose) {
            body.setPose(pose.toIsometry());
          },
          py::arg("pose"), DOC(mplib, AttachedBodyTpl, setPose))
      .def(
          "get_attached_link_global_pose",
          [](const AttachedBody &body) {
            return Pose<S>(body.getAttachedLinkGlobalPose());
          },
          DOC(mplib, AttachedBodyTpl, getAttachedLinkGlobalPose))
      .def(
          "get_global_pose",
          [](const AttachedBody &body) { return Pose<S>(body.getGlobalPose()); },
          DOC(mplib, AttachedBodyTpl, getGlobalPose))

      .def("update_pose", &AttachedBody::updatePose,
           DOC(mplib, AttachedBodyTpl, updatePose))
      .def("get_touch_links", &AttachedBody::getTouchLinks,
           DOC(mplib, AttachedBodyTpl, getTouchLinks))
      .def("set_touch_links", &AttachedBody::setTouchLinks, py::arg("touch_links"),
           DOC(mplib, AttachedBodyTpl, setTouchLinks));
}

}  // namespace mplib
