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
using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;

void build_pyattached_body(py::module &pymp) {
  auto PyAttachedBody = py::class_<AttachedBody, std::shared_ptr<AttachedBody>>(
      pymp, "AttachedBody", DOC(mplib, AttachedBodyTpl));
  PyAttachedBody
      .def(py::init([](const std::string &name, const CollisionObjectPtr &object,
                       const ArticulatedModelPtr &attached_articulation,
                       int attached_link_id, const Vector7<S> &posevec,
                       const std::vector<std::string> &touch_links) {
             Isometry3<S> pose;
             pose.linear() =
                 Quaternion<S>(posevec[3], posevec[4], posevec[5], posevec[6]).matrix();
             pose.translation() = posevec.head(3);
             return AttachedBody(name, object, attached_articulation, attached_link_id,
                                 pose, touch_links);
           }),
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
      .def("get_pose", &AttachedBody::getPose, DOC(mplib, AttachedBodyTpl, getPose))
      .def(
          "set_pose",
          [](AttachedBody &self, const Vector7<S> &posevec) {
            Isometry3<S> pose;
            pose.linear() =
                Quaternion<S>(posevec[3], posevec[4], posevec[5], posevec[6]).matrix();
            pose.translation() = posevec.head(3);
            self.setPose(pose);
          },
          py::arg("pose"), DOC(mplib, AttachedBodyTpl, setPose))
      .def("get_global_pose", &AttachedBody::getGlobalPose,
           DOC(mplib, AttachedBodyTpl, getGlobalPose))
      .def("update_pose", &AttachedBody::updatePose,
           DOC(mplib, AttachedBodyTpl, updatePose))
      .def("get_touch_links", &AttachedBody::getTouchLinks,
           DOC(mplib, AttachedBodyTpl, getTouchLinks))
      .def("set_touch_links", &AttachedBody::setTouchLinks, py::arg("touch_links"),
           DOC(mplib, AttachedBodyTpl, setTouchLinks));
}

}  // namespace mplib
