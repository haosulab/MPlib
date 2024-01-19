#pragma once

#include "color_printing.h"
#include "fcl_model.h"
#include "macros_utils.hpp"
#include "pinocchio_model.h"

/**
 * Supports initialization from URDF and SRDF files, and provides access to
 * underlying Pinocchio and FCL models.
 */
template <typename DATATYPE>
class ArticulatedModelTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(DATATYPE);
  using PinocchioModel = PinocchioModelTpl<DATATYPE>;
  using FCLModel = FCLModelTpl<DATATYPE>;

  PinocchioModel pinocchio_model;
  FCLModel fcl_model;

  std::vector<std::string> user_link_names;
  std::vector<std::string>
      user_joint_names;  // all links and joints you want to control. order matters

  std::vector<size_t> move_group_user_joints;
  std::vector<std::string> move_group_end_effectors;
  VectorX current_qpos;  // The planning world only update the state in planning group.

  size_t move_group_qpos_dim;
  bool verbose;

  // the base pose of the robot
  Vector7 base_pose;
  Transform3 base_tf;

 public:
  /**
   * Construct an articulated model from URDF and SRDF files.
   *
   * @param urdf_filename: path to URDF file, can be relative to the current working
   *  directory
   * @param srdf_filename: path to SRDF file, we use it to disable self-collisions
   * @param gravity: gravity vector
   * @param joint_names: list of joints that are considered for planning
   * @param link_names: list of links that are considered for planning
   * @param verbose: print debug information
   * @param convex: use convex decomposition for collision objects
   */
  ArticulatedModelTpl(const std::string &urdf_filename,
                      const std::string &srdf_filename, const Vector3 &gravity,
                      const std::vector<std::string> &joint_names = {},
                      const std::vector<std::string> &link_names = {},
                      const bool &verbose = true, const bool &convex = false);

  /**
   * Get the underlying Pinocchio model.
   *
   * @return: Pinocchio model used for kinematics and dynamics computations
   */
  PinocchioModelTpl<DATATYPE> &getPinocchioModel() { return pinocchio_model; }

  /**
   * Get the underlying FCL model.
   *
   * @return: FCL model used for collision checking
   */
  FCLModelTpl<DATATYPE> &getFCLModel() { return fcl_model; }

  /**
   * Set the move group, i.e. the chain ending in end effector for which to compute the
   *  forward kinematics for all subsequent queries.
   *
   * @param chain: list of links extending to the end effector
   */
  void setMoveGroup(const std::string &end_effector);

  /**
   * Set the move group but we have multiple end effectors in a chain.
   * I.e., Base --> EE1 --> EE2 --> ... --> EEn
   *
   * @param end_effectors: names of the end effector link
   */
  void setMoveGroup(const std::vector<std::string> &end_effectors);

  /**
   * Get the joint indices of the move group.
   *
   * @return: list of user joint indices of the move group
   */
  std::vector<size_t> getMoveGroupJointIndices(void) { return move_group_user_joints; }

  /**
   * Get the joint names of the move group.
   *
   * @return: list of joint names of the move group
   */
  std::vector<std::string> getMoveGroupJointName(void);

  /**
   * Get the joint names that the user has provided for planning.
   *
   * @return: list of joint names of the user
   */
  std::vector<std::string> getUserJointNames(void) { return user_joint_names; }

  /**
   * Get the link names that the user has provided for planning.
   *
   * @return: list of link names of the user
   */
  std::vector<std::string> getUserLinkNames(void) { return user_link_names; }

  /**
   * Get the end effectors of the move group.
   *
   * @return: list of end effectors of the move group
   */
  std::vector<std::string> getMoveGroupEndEffectors(void) {
    return move_group_end_effectors;
  }

  /**
   * Get the dimension of the move group qpos.
   *
   * @return: dimension of the move group qpos
   */
  size_t getQposDim(void) { return move_group_qpos_dim; }

  /**
   * Get the current joint position of all active joints inside the URDF.
   *
   * @return: current qpos of all active joints
   */
  VectorX getQpos(void) { return current_qpos; }

  /**
   * Let the planner know the current joint positions.
   *
   * @param qpos: current qpos of all active joints or just the move group joints
   * @param full: whether to set the full qpos or just the move group qpos.
   *    If full is ``false``, we will pad the missing joints with current known qpos.
   *    The default is ``false``
   */
  void setQpos(const VectorX &qpos, const bool &full = false);

  /** Only support one end effector case */
  size_t getEEFrameIndex() {
    return std::find(user_link_names.begin(), user_link_names.end(),
                     move_group_end_effectors[0]) -
           user_link_names.begin();
  }

  /**
   * Update the SRDF file to disable self-collisions.
   *
   * @param srdf: path to SRDF file, can be relative to the current working directory
   */
  void updateSRDF(const std::string &srdf) {
    fcl_model.removeCollisionPairsFromSrdf(srdf);
  }

  /**
   * Set the base pose of the robot.
   *
   * @param pose: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
   */
  void setBasePose(const Vector7 &pose);

  /**
   * Get the base pose of the robot.
   *
   * @return: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
   */
  Vector7 getBasePose() { return base_pose; }
};

template <typename T>
using ArticulatedModelTpl_ptr = std::shared_ptr<ArticulatedModelTpl<T>>;

using ArticulatedModeld = ArticulatedModelTpl<double>;
using ArticulatedModelf = ArticulatedModelTpl<float>;
using ArticulatedModeld_ptr = ArticulatedModelTpl_ptr<double>;
using ArticulatedModelf_ptr = ArticulatedModelTpl_ptr<float>;
