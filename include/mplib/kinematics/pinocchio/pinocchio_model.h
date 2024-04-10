#pragma once

#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "mplib/kinematics/pinocchio/types.h"
#include "mplib/macros/class_forward.h"
#include "mplib/types.h"
#include "mplib/utils/pose.h"

namespace mplib::kinematics::pinocchio {

// PinocchioModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(PinocchioModelTpl);

/**
 * Pinocchio model of an articulation
 *
 * See https://github.com/stack-of-tasks/pinocchio
 */
template <typename S>
class PinocchioModelTpl {
 public:
  /**
   * Construct a Pinocchio model from the given URDF file.
   *
   * @param urdf_filename: path to the URDF file
   * @param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
   * @param verbose: print debug information. Default: ``false``.
   */
  PinocchioModelTpl(const std::string &urdf_filename,
                    const Vector3<S> &gravity = Vector3<S> {0, 0, -9.81},
                    bool verbose = false);

  /**
   * Construct a Pinocchio model from the given URDF file.
   *
   * @param urdf_model: a urdf tree as urdf::ModelInterfaceSharedPtr
   * @param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
   * @param verbose: print debug information. Default: ``false``.
   */
  PinocchioModelTpl(const urdf::ModelInterfaceSharedPtr &urdf_model,
                    const Vector3<S> &gravity = Vector3<S> {0, 0, -9.81},
                    bool verbose = false);

  /**
   * Constructs a PinocchioModel from URDF string
   *
   * @param urdf_string: URDF string (without visual/collision elements for links)
   * @param gravity: gravity vector, by default is ``[0, 0, -9.81]`` in -z axis
   * @param verbose: print debug information. Default: ``false``.
   * @return: a unique_ptr to PinocchioModel
   */
  static std::unique_ptr<PinocchioModelTpl<S>> createFromURDFString(
      const std::string &urdf_string,
      const Vector3<S> &gravity = Vector3<S> {0, 0, -9.81}, bool verbose = false);

  /// Get the underlying pinocchio::Model
  const pinocchio::ModelTpl<S> &getModel() const { return model_; }

  /// Get the underlying pinocchio::Data
  const pinocchio::DataTpl<S> &getData() const { return data_; }

  /**
   * Get the leaf links (links without child) of the kinematic tree.
   *
   * @return: list of leaf links
   */
  const std::vector<std::string> &getLeafLinks() const { return leaf_links_; }

  /**
   * Get the all adjacent link names.
   *
   * @return: adjacent link names as a set of pairs of strings
   */
  std::set<std::pair<std::string, std::string>> getAdjacentLinks() const;

  /**
   * Pinocchio might have a different link order or it might add additional links.
   *
   * If you do not pass the the list of link names, the default order might not be the
   * one you want.
   *
   * @param names: list of link names in the order you want
   */
  void setLinkOrder(const std::vector<std::string> &names);

  /**
   * Pinocchio might have a different joint order or it might add additional joints.
   *
   * If you do not pass the the list of joint names, the default order might not be the
   * one you want.
   *
   * @param names: list of joint names in the order you want
   */
  void setJointOrder(const std::vector<std::string> &names);

  /**
   * Get the name of all the links.
   *
   * @param user: if ``true``, we get the name of the links in the order you passed
   *    to the constructor or the default order
   * @return: name of the links
   */
  std::vector<std::string> getLinkNames(bool user = true) const;

  /**
   * Get the name of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the name of the joints in the order you passed
   *    to the constructor or the default order
   * @return: name of the joints
   */
  std::vector<std::string> getJointNames(bool user = true) const;

  /**
   * Get the id of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: id of the joint with the given index
   */
  size_t getJointId(size_t index, bool user = true) const {
    return user ? vidx_[index] : model_.idx_vs[index];
  }

  /**
   * Get the id of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the id of the joints in the order you passed to
   *    the constructor or the default order
   * @return: id of the joints
   */
  VectorXi getJointIds(bool user = true) const;

  /**
   * Get the type of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: type of the joint with the given index
   */
  std::string getJointType(size_t index, bool user = true) const;

  /**
   * Get the type of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the type of the joints in the order you passed to
   *    the constructor or the default order
   * @return: type of the joints
   */
  std::vector<std::string> getJointTypes(bool user = true) const;

  /**
   * Get the dimension of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: dimension of the joint with the given index
   */
  size_t getJointDim(size_t index, bool user = true) const {
    return user ? nvs_[index] : model_.nvs[index];
  }

  /**
   * Get the dimension of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the dimension of the joints in the order you
   *    passed to the constructor or the default order
   * @return: dimention of the joints
   */
  VectorXi getJointDims(bool user = true) const;

  /**
   * Get the limit of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: limit of the joint with the given index
   */
  MatrixX<S> getJointLimit(size_t index, bool user = true) const;

  /**
   * Get the limit of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the limit of the joints in the order you passed to
   *    the constructor or the default order
   * @return: limit of the joints
   */
  std::vector<MatrixX<S>> getJointLimits(bool user = true) const;

  /**
   * Get the parent of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: parent of the joint with the given index
   */
  size_t getJointParent(size_t index, bool user = true) const {
    return user ? parents_[index] : model_.parents[index];
  }

  /**
   * Get the parent of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the parent of the joints in the order you passed
   *    to the constructor or the default order
   * @return: parent of the joints
   */
  VectorXi getJointParents(bool user = true) const;

  /// Frame is a Pinocchio internal data type which is not supported outside this class.
  void printFrames() const;

  /**
   * Get the joint names of the joints in the chain from the root to the given link.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @return: joint names of the joints in the chain
   */
  std::vector<std::string> getChainJointName(const std::string &end_effector) const;

  /**
   * Get the joint indices of the joints in the chain from the root to the given link.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @return: joint indices of the joints in the chain
   */
  std::vector<std::size_t> getChainJointIndex(const std::string &end_effector) const;

  /**
   * Get a random configuration.
   *
   * @return: random joint configuration
   */
  VectorX<S> getRandomConfiguration() const;

  /**
   * Compute forward kinematics for the given joint configuration.
   *
   * If you want the result you need to call ``get_link_pose()``
   *
   * @param qpos: joint configuration. Needs to be full configuration, not just the
   *    movegroup joints.
   */
  void computeForwardKinematics(const VectorX<S> &qpos);

  /**
   * Get the pose of the given link in robot's base (root) frame.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @return: pose of the link in robot's base (root) frame.
   */
  Isometry3<S> getLinkPose(size_t index) const;

  Isometry3<S> getJointPose(size_t index) const;  // TODO: not same as sapien

  /**
   * Compute the full jacobian for the given joint configuration.
   * Note you need to call computeForwardKinematics() first.
   * If you want the result you need to call ``get_link_jacobian()``
   *
   * @param qpos: joint configuration. Needs to be full configuration, not just the
   *    movegroup joints.
   */
  void computeFullJacobian(const VectorX<S> &qpos);

  /**
   * Get the jacobian of the given link. You must call ``compute_full_jacobian()``
   * first.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param local: if ``true``, the jacobian is expressed in the local frame of the
   *    link, otherwise it is expressed in the world frame
   * @return: 6 x n jacobian of the link
   */
  Matrix6X<S> getLinkJacobian(size_t index, bool local = false) const;

  /**
   * Compute the jacobian of the given link.
   * Note you need to call computeForwardKinematics() first.
   *
   * @param qpos: joint configuration. Needs to be full configuration, not just the
   *    movegroup joints.
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param local: if ``true`` return the jacobian w.r.t. the instantaneous local frame
   *    of the link
   * @return: 6 x n jacobian of the link
   */
  Matrix6X<S> computeSingleLinkJacobian(const VectorX<S> &qpos, size_t index,
                                        bool local = false);

  /**
   * Compute the inverse kinematics using close loop inverse kinematics.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param pose: desired pose of the link
   * @param q_init: initial joint configuration
   * @param mask: if the value at a given index is ``true``, the joint is *not* used in
   *    the IK
   * @param eps: tolerance for the IK
   * @param max_iter: maximum number of iterations
   * @param dt: time step for the CLIK
   * @param damp: damping for the CLIK
   * @return: joint configuration
   */
  std::tuple<VectorX<S>, bool, Vector6<S>> computeIKCLIK(
      size_t index, const Pose<S> &pose, const VectorX<S> &q_init,
      const std::vector<bool> &mask, double eps = 1e-5, int max_iter = 1000,
      double dt = 1e-1, double damp = 1e-12);

  /**
   * The same as ``compute_IK_CLIK()`` but with it clamps the joint configuration to the
   * given limits.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param pose: desired pose of the link
   * @param q_init: initial joint configuration
   * @param q_min: minimum joint configuration
   * @param q_max: maximum joint configuration
   * @param eps: tolerance for the IK
   * @param max_iter: maximum number of iterations
   * @param dt: time step for the CLIK
   * @param damp: damping for the CLIK
   * @return: joint configuration
   */
  std::tuple<VectorX<S>, bool, Vector6<S>> computeIKCLIKJL(
      size_t index, const Pose<S> &pose, const VectorX<S> &q_init,
      const VectorX<S> &qmin, const VectorX<S> &qmax, double eps = 1e-5,
      int max_iter = 1000, double dt = 1e-1, double damp = 1e-12);

 private:
  void init(const urdf::ModelInterfaceSharedPtr &urdf_model, const Vector3<S> &gravity);

  void dfsParseTree(const urdf::LinkConstSharedPtr &link,
                    pinocchio::UrdfVisitorBase<S> &visitor);

  VectorX<S> qposUser2Pinocchio(const VectorX<S> &q_user) const;

  VectorX<S> qposPinocchio2User(const VectorX<S> &q_pinocchio) const;

  urdf::ModelInterfaceSharedPtr urdf_model_;
  pinocchio::ModelTpl<S> model_;
  pinocchio::DataTpl<S> data_;
  std::vector<std::string> leaf_links_;

  std::vector<std::string> user_link_names_;
  VectorXi link_index_user2pinocchio_;

  std::vector<std::string> user_joint_names_;
  VectorXi v_index_user2pinocchio_;  // the joint index in model
  VectorXi vidx_, qidx_, nvs_, nqs_, parents_;
  VectorXi joint_index_user2pinocchio_, joint_index_pinocchio2user_;
  PermutationMatrixX v_map_user2pinocchio_;  // map between user and pinocchio

  bool verbose_ {};
};

// Common Type Alias ===================================================================
using PinocchioModelf = PinocchioModelTpl<float>;
using PinocchioModeld = PinocchioModelTpl<double>;
using PinocchioModelfPtr = PinocchioModelTplPtr<float>;
using PinocchioModeldPtr = PinocchioModelTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_PINOCCHIO_MODEL(S) extern template class PinocchioModelTpl<S>

DECLARE_TEMPLATE_PINOCCHIO_MODEL(float);
DECLARE_TEMPLATE_PINOCCHIO_MODEL(double);

}  // namespace mplib::kinematics::pinocchio
