#pragma once

#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../src/articulated_model.h"

namespace py = pybind11;

using DATATYPE = double;

using ArticulatedModel = ArticulatedModelTpl<DATATYPE>;

std::string constructor_doc = R"(
    Construct an articulated model from URDF and SRDF files.
    
    Args:
        urdf_filename: path to URDF file, can be relative to the current working directory
        srdf_filename: path to SRDF file, we use it to disable self-collisions
        gravity: gravity vector
        joint_names: list of joints that are considered for planning
        link_names: list of links that are considered for planning
        verbose: print debug information
        convex: use convex decomposition for collision objects)";

std::string get_pinnochio_model_doc = R"(
    Get the underlying Pinocchio model.
    
    Returns:
        Pinocchio model used for kinematics and dynamics computations)";

std::string get_fcl_model_doc = R"(
    Get the underlying FCL model.
    
    Returns:
        FCL model used for collision checking)";

std::string set_move_group_doc1 = R"(
    Set the move group, i.e. the chain ending in end effector for which to compute the forward kinematics for all subsequent queries.
    
    Args:
        chain: list of links extending to the end effector)";

std::string set_move_group_doc2 = R"(
    Set the move group but we have multiple end effectors in a chain. i.e. Base --> EE1 --> EE2 --> ... --> EEn
    
    Args:
        end_effectors: names of the end effector link)";

std::string get_move_group_joint_indices_doc = R"(
    Get the joint indices of the move group.
    
    Returns:
        list of user joint indices of the move group)";

std::string get_move_group_joint_names_doc = R"(
    Get the joint names of the move group.
    
    Returns:
        list of joint names of the move group)";

std::string get_user_joint_names_doc = R"(
    Get the joint names that the user has provided for planning.
    
    Returns:
        list of joint names of the user)";

std::string get_user_link_names_doc = R"(
    Get the link names that the user has provided for planning.
    
    Returns:
        list of link names of the user)";

std::string get_move_group_end_effectors_doc = R"(
    Get the end effectors of the move group.
    
    Returns:
        list of end effectors of the move group)";

std::string get_qpos_doc = R"(
    Get the current joint position of all active joints inside the URDF.
    
    Returns:
        current qpos of all active joints)";

std::string get_move_group_qpos_dim_doc = R"(
    Get the dimension of the move group qpos.
    
    Returns:
        dimension of the move group qpos)";

std::string set_qpos_doc = R"(
    Let the planner know the current joint positions.
    
    Args:
        qpos: current qpos of all active joints or just the move group joints
        full: whether to set the full qpos or just the move group qpos. if full false, we will pad the missing joints with current known qpos. the default is false)";

std::string update_srdf_doc = R"(
    Update the SRDF file to disable self-collisions.
    
    Args:
        srdf: path to SRDF file, can be relative to the current working directory)";

std::string set_base_pose_doc = R"(
    Set the base pose of the robot.
    
    Args:
        pose: base pose of the robot in [x, y, z, qw, qx, qy, qz] format)";

std::string get_base_pose_doc = R"(
    Get the base pose of the robot.
    
    Returns:
        base pose of the robot in [x, y, z, qw, qx, qy, qz] format)";

void build_pyarticulation(py::module &m_all) {
  auto m = m_all.def_submodule(
      "articulation", "articulated model submodule, i.e. models with moving parts");
  auto PyArticulatedModel =
      py::class_<ArticulatedModel, std::shared_ptr<ArticulatedModel>>(
          m, "ArticulatedModel",
          "Supports initialization from URDF and SRDF files, and provides access to "
          "underlying Pinocchio and FCL models.");

  PyArticulatedModel
      .def(py::init<const std::string &, const std::string &,
                    Eigen::Matrix<DATATYPE, 3, 1>, const std::vector<std::string> &,
                    const std::vector<std::string> &, const bool &, const bool &>(),
           py::arg("urdf_filename"), py::arg("srdf_filename"), py::arg("gravity"),
           py::arg("joint_names"), py::arg("link_names"), py::arg("verbose") = true,
           py::arg("convex") = false, constructor_doc.c_str())
      .def("get_pinocchio_model", &ArticulatedModel::getPinocchioModel,
           get_pinnochio_model_doc.c_str())
      .def("get_fcl_model", &ArticulatedModel::getFCLModel, get_fcl_model_doc.c_str())
      .def("set_move_group",
           py::overload_cast<const std::string &>(&ArticulatedModel::setMoveGroup),
           py::arg("end_effector"), set_move_group_doc1.c_str())
      .def("set_move_group",
           py::overload_cast<const std::vector<std::string> &>(
               &ArticulatedModel::setMoveGroup),
           py::arg("end_effectors"), set_move_group_doc2.c_str())
      .def("set_base_pose", &ArticulatedModel::setBasePose, py::arg("pose"),
           set_base_pose_doc.c_str())
      .def("get_base_pose", &ArticulatedModel::getBasePose, get_base_pose_doc.c_str())
      .def("get_move_group_joint_indices", &ArticulatedModel::getMoveGroupJointIndices,
           get_move_group_joint_indices_doc.c_str())
      .def("get_move_group_joint_names", &ArticulatedModel::getMoveGroupJointName,
           get_move_group_joint_names_doc.c_str())
      .def("get_user_joint_names", &ArticulatedModel::getUserJointNames,
           get_user_joint_names_doc.c_str())
      .def("get_user_link_names", &ArticulatedModel::getUserLinkNames,
           get_user_link_names_doc.c_str())
      .def("get_move_group_end_effectors", &ArticulatedModel::getMoveGroupEndEffectors,
           get_move_group_end_effectors_doc.c_str())
      .def("get_qpos", &ArticulatedModel::getQpos, get_qpos_doc.c_str())
      .def("get_move_group_qpos_dim", &ArticulatedModel::getQposDim,
           get_move_group_qpos_dim_doc.c_str())
      .def("set_qpos", &ArticulatedModel::setQpos, py::arg("qpos"),
           py::arg("full") = false, set_qpos_doc.c_str())
      .def("update_SRDF", &ArticulatedModel::updateSRDF, py::arg("SRDF"),
           update_srdf_doc.c_str());
}
