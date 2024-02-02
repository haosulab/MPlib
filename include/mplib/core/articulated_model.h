#pragma once

#include <string>
#include <vector>

#include "mplib/collision_detection/types.h"
#include "mplib/kinematics/types.h"
#include "mplib/macros/class_forward.h"
#include "mplib/types.h"
#include "mplib/utils/conversion.h"

namespace mplib {

// ArticulatedModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(ArticulatedModelTpl);

/**
 * Supports initialization from URDF and SRDF files, and provides access to
 * underlying Pinocchio and FCL models.
 */
template <typename S>
class ArticulatedModelTpl {
 public:
  /**
   * Construct an articulated model from URDF and SRDF files.
   *
   * @param urdf_filename: path to URDF file, can be relative to the current working
   *  directory
   * @param srdf_filename: path to SRDF file, we use it to disable self-collisions
   * @param gravity: gravity vector
   * @param link_names: list of links that are considered for planning
   * @param joint_names: list of joints that are considered for planning
   * @param convex: use convex decomposition for collision objects. Default: ``false``.
   * @param verbose: print debug information. Default: ``false``.
   */
  ArticulatedModelTpl(const std::string &urdf_filename,
                      const std::string &srdf_filename, const Vector3<S> &gravity,
                      const std::vector<std::string> &link_names = {},
                      const std::vector<std::string> &joint_names = {},
                      bool convex = false, bool verbose = false);

  /**
   * Get the underlying Pinocchio model.
   *
   * @return: Pinocchio model used for kinematics and dynamics computations
   */
  kinematics::PinocchioModelTplPtr<S> getPinocchioModel() const {
    return pinocchio_model_;
  }

  /**
   * Get the underlying FCL model.
   *
   * @return: FCL model used for collision checking
   */
  collision_detection::FCLModelTplPtr<S> getFCLModel() const { return fcl_model_; }

  /**
   * Get the link names that the user has provided for planning.
   *
   * @return: list of link names of the user
   */
  const std::vector<std::string> &getUserLinkNames() const { return user_link_names_; }

  /**
   * Get the joint names that the user has provided for planning.
   *
   * @return: list of joint names of the user
   */
  const std::vector<std::string> &getUserJointNames() const {
    return user_joint_names_;
  }

  /**
   * Get the end effectors of the move group.
   *
   * @return: list of end effectors of the move group
   */
  const std::vector<std::string> &getMoveGroupEndEffectors() const {
    return move_group_end_effectors_;
  }

  /**
   * Get the joint indices of the move group.
   *
   * @return: list of user joint indices of the move group
   */
  const std::vector<size_t> &getMoveGroupJointIndices() const {
    return move_group_user_joints_;
  }

  /**
   * Get the joint names of the move group.
   *
   * @return: list of joint names of the move group
   */
  std::vector<std::string> getMoveGroupJointNames() const;

  /**
   * Set the move group, i.e. the chain ending in end effector for which to compute the
   * forward kinematics for all subsequent queries.
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
   * Get the dimension of the move group qpos.
   *
   * @return: dimension of the move group qpos
   */
  size_t getQposDim() const { return move_group_qpos_dim_; }

  /**
   * Get the current joint position of all active joints inside the URDF.
   *
   * @return: current qpos of all active joints
   */
  const VectorX<S> &getQpos() const { return current_qpos_; }

  /**
   * Let the planner know the current joint positions.
   *
   * @param qpos: current qpos of all active joints or just the move group joints
   * @param full: whether to set the full qpos or just the move group qpos.
   *    If full is ``false``, we will pad the missing joints with current known qpos.
   *    The default is ``false``
   */
  void setQpos(const VectorX<S> &qpos, bool full = false);

  /**
   * Get the base pose of the robot.
   *
   * @return: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
   */
  Vector7<S> getBasePose() const { return toPoseVec<S>(base_pose_); }

  /**
   * Set the base pose of the robot.
   *
   * @param pose: base pose of the robot in [x, y, z, qw, qx, qy, qz] format
   */
  void setBasePose(const Vector7<S> &pose);

  /**
   * Update the SRDF file to disable self-collisions.
   *
   * @param srdf: path to SRDF file, can be relative to the current working directory
   */
  void updateSRDF(const std::string &srdf) {
    fcl_model_->removeCollisionPairsFromSRDF(srdf);
  }

 private:
  kinematics::PinocchioModelTplPtr<S> pinocchio_model_;
  collision_detection::FCLModelTplPtr<S> fcl_model_;

  // all links and joints you want to control. order matters
  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;

  // The planning world only update the state in planning group.
  std::vector<std::string> move_group_end_effectors_;
  std::vector<size_t> move_group_user_joints_;
  size_t move_group_qpos_dim_;
  VectorX<S> current_qpos_;

  // the base pose of the robot
  Isometry3<S> base_pose_;

  bool verbose_;
};

// Common Type Alias ===================================================================
using ArticulatedModelf = ArticulatedModelTpl<float>;
using ArticulatedModeld = ArticulatedModelTpl<double>;
using ArticulatedModelfPtr = ArticulatedModelTplPtr<float>;
using ArticulatedModeldPtr = ArticulatedModelTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_ARTICULATED_MODEL(S) \
  extern template class ArticulatedModelTpl<S>

DECLARE_TEMPLATE_ARTICULATED_MODEL(float);
DECLARE_TEMPLATE_ARTICULATED_MODEL(double);

}  // namespace mplib
