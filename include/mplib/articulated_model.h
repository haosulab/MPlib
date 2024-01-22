#pragma once

#include "color_printing.h"
#include "fcl_model.h"
#include "macros_utils.h"
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

  PinocchioModel pinocchio_model_;
  FCLModel fcl_model_;

  std::vector<std::string> user_link_names_;
  std::vector<std::string>
      user_joint_names_;  // all links and joints you want to control. order matters

  std::vector<size_t> move_group_user_joints_;
  std::vector<std::string> move_group_end_effectors_;
  VectorX current_qpos_;  // The planning world only update the state in planning group.

  size_t move_group_qpos_dim_;
  bool verbose_;

  // the base pose of the robot
  Vector7 base_pose_;
  Transform3 base_tf_;

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
  PinocchioModelTpl<DATATYPE> &getPinocchioModel() { return pinocchio_model_; }

  /**
   * Get the underlying FCL model.
   *
   * @return: FCL model used for collision checking
   */
  FCLModelTpl<DATATYPE> &getFCLModel() { return fcl_model_; }

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
  std::vector<size_t> getMoveGroupJointIndices(void) { return move_group_user_joints_; }

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
  std::vector<std::string> getUserJointNames(void) { return user_joint_names_; }

  /**
   * Get the link names that the user has provided for planning.
   *
   * @return: list of link names of the user
   */
  std::vector<std::string> getUserLinkNames(void) { return user_link_names_; }

  /**
   * Get the end effectors of the move group.
   *
   * @return: list of end effectors of the move group
   */
  std::vector<std::string> getMoveGroupEndEffectors(void) {
    return move_group_end_effectors_;
  }

  /**
   * Get the dimension of the move group qpos.
   *
   * @return: dimension of the move group qpos
   */
  size_t getQposDim(void) { return move_group_qpos_dim_; }

  /**
   * Get the current joint position of all active joints inside the URDF.
   *
   * @return: current qpos of all active joints
   */
  VectorX getQpos(void) { return current_qpos_; }

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
    return std::find(user_link_names_.begin(), user_link_names_.end(),
                     move_group_end_effectors_[0]) -
           user_link_names_.begin();
  }

  /**
   * Update the SRDF file to disable self-collisions.
   *
   * @param srdf: path to SRDF file, can be relative to the current working directory
   */
  void updateSRDF(const std::string &srdf) {
    fcl_model_.removeCollisionPairsFromSrdf(srdf);
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
  Vector7 getBasePose() { return base_pose_; }
};

template <typename T>
using ArticulatedModelTplPtr = std::shared_ptr<ArticulatedModelTpl<T>>;

using ArticulatedModeld = ArticulatedModelTpl<double>;
using ArticulatedModelf = ArticulatedModelTpl<float>;
using ArticulatedModeldPtr = ArticulatedModelTplPtr<double>;
using ArticulatedModelfPtr = ArticulatedModelTplPtr<float>;
