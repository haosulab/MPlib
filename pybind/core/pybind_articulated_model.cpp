#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "docstring/core/articulated_model.h"
#include "mplib/core/articulated_model.h"
#include "pybind_macros.hpp"

namespace py = pybind11;

namespace mplib {

using ArticulatedModel = ArticulatedModelTpl<S>;

void build_pyarticulated_model(py::module &pymp) {
  auto PyArticulatedModel =
      py::class_<ArticulatedModel, std::shared_ptr<ArticulatedModel>>(
          pymp, "ArticulatedModel", DOC(mplib, ArticulatedModelTpl));

  PyArticulatedModel
      .def(py::init<const std::string &, const std::string &, const Vector3<S> &,
                    const std::vector<std::string> &, const std::vector<std::string> &,
                    bool, bool>(),
           py::arg("urdf_filename"), py::arg("srdf_filename"), py::arg("gravity"),
           py::arg("link_names"), py::arg("joint_names"), py::arg("convex") = false,
           py::arg("verbose") = false,
           DOC(mplib, ArticulatedModelTpl, ArticulatedModelTpl))

      .def("get_name", &ArticulatedModel::getName,
           DOC(mplib, ArticulatedModelTpl, getName))
      .def("set_name", &ArticulatedModel::setName, py::arg("name"),
           DOC(mplib, ArticulatedModelTpl, setName))

      .def("get_pinocchio_model", &ArticulatedModel::getPinocchioModel,
           DOC(mplib, ArticulatedModelTpl, getPinocchioModel))
      .def("get_fcl_model", &ArticulatedModel::getFCLModel,
           DOC(mplib, ArticulatedModelTpl, getFCLModel))

      .def("get_user_link_names", &ArticulatedModel::getUserLinkNames,
           DOC(mplib, ArticulatedModelTpl, getUserLinkNames))
      .def("get_user_joint_names", &ArticulatedModel::getUserJointNames,
           DOC(mplib, ArticulatedModelTpl, getUserJointNames))

      .def("get_move_group_end_effectors", &ArticulatedModel::getMoveGroupEndEffectors,
           DOC(mplib, ArticulatedModelTpl, getMoveGroupEndEffectors))
      .def("get_move_group_joint_indices", &ArticulatedModel::getMoveGroupJointIndices,
           DOC(mplib, ArticulatedModelTpl, getMoveGroupJointIndices))
      .def("get_move_group_joint_names", &ArticulatedModel::getMoveGroupJointNames,
           DOC(mplib, ArticulatedModelTpl, getMoveGroupJointNames))
      .def("set_move_group",
           py::overload_cast<const std::string &>(&ArticulatedModel::setMoveGroup),
           py::arg("end_effector"), DOC(mplib, ArticulatedModelTpl, setMoveGroup))
      .def("set_move_group",
           py::overload_cast<const std::vector<std::string> &>(
               &ArticulatedModel::setMoveGroup),
           py::arg("end_effectors"), DOC(mplib, ArticulatedModelTpl, setMoveGroup, 2))

      .def("get_move_group_qpos_dim", &ArticulatedModel::getQposDim,
           DOC(mplib, ArticulatedModelTpl, getQposDim))
      .def("get_qpos", &ArticulatedModel::getQpos,
           DOC(mplib, ArticulatedModelTpl, getQpos))
      .def("set_qpos", &ArticulatedModel::setQpos, py::arg("qpos"),
           py::arg("full") = false, DOC(mplib, ArticulatedModelTpl, setQpos))

      .def("get_base_pose", &ArticulatedModel::getBasePose,
           DOC(mplib, ArticulatedModelTpl, getBasePose))
      .def("set_base_pose", &ArticulatedModel::setBasePose, py::arg("pose"),
           DOC(mplib, ArticulatedModelTpl, setBasePose))

      .def("update_SRDF", &ArticulatedModel::updateSRDF, py::arg("SRDF"),
           DOC(mplib, ArticulatedModelTpl, updateSRDF));
}

}  // namespace mplib
