#pragma once
#include <vector>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "color_printing.h"
#include "macros_utils.h"

/**
 * Pinocchio model of an articulation
 *
 * See https://github.com/stack-of-tasks/pinocchio
 */
template <typename DATATYPE>
class PinocchioModelTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(DATATYPE)
  DEFINE_TEMPLATE_PINOCCHIO(DATATYPE)

  // pinocchio::ModelTpl<float>;

  urdf::ModelInterfaceSharedPtr urdf_model_;
  Model model_;
  Data data_;

  VectorXI joint_index_user2pinocchio_, joint_index_pinocchio2user_;
  VectorXI v_index_user2pinocchio_;  // the joint index in model
  // map between user and pinocchio
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_map_user2pinocchio_;
  VectorXI link_index_user2pinocchio_;

  std::vector<std::string> user_link_names_;
  std::vector<std::string> user_joint_names_;
  std::vector<std::string> leaf_links_;
  VectorXI qidx_, vidx_, nqs_, nvs_, parents_;

  const std::string joint_prefix_ = "JointModel";
  bool verbose_;

  VectorX qposUser2Pinocchio(const VectorX &qpos);

  VectorX qposPinocchio2User(const VectorX &qpos);

  void init(const urdf::ModelInterfaceSharedPtr &urdfTree, const Vector3 &gravity);

  void dfs_parse_tree(urdf::LinkConstSharedPtr link, UrdfVisitorBase &visitor);

 public:
  PinocchioModelTpl(const urdf::ModelInterfaceSharedPtr &urdfTree,
                    const Vector3 &gravity, const bool &verbose = true);

  /**
   * Construct a Pinocchio model from the given URDF file.
   *
   * @param urdf_filename: path to the URDF file
   * @param gravity: gravity vector
   * @param verbose: print debug information
   */
  PinocchioModelTpl(const std::string &urdf_filename, const Vector3 &gravity,
                    const bool &verbose = true);

  inline Model &getModel(void) { return model_; }

  inline Data &getData(void) { return data_; }

  /**
   * Get the leaf links (links without child) of the kinematic tree.
   *
   * @return: list of leaf links
   */
  std::vector<std::string> getLeafLinks() { return leaf_links_; }

  /**
   * Get the type of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: type of the joint with the given index
   */
  inline std::string getJointType(const size_t &index, const bool &user = true) {
    if (user)
      return model_.joints[joint_index_user2pinocchio_[index]].shortname();
    else
      return model_.joints[index].shortname();
  }

  /**
   * Get the type of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the type of the joints in the order you passed to
   *    the constructor or the default order
   * @return: type of the joints
   */
  inline std::vector<std::string> getJointTypes(const bool &user = true) {
    std::vector<std::string> ret;
    auto njoints = user ? user_joint_names_.size() : model_.joints.size();
    for (size_t i = 0; i < njoints; i++) ret.push_back(getJointType(i, user));
    return ret;
  }

  /**
   * Get the limit of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the limit of the joints in the order you passed to
   *    the constructor or the default order
   * @return: limit of the joints
   */
  inline std::vector<Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
  getJointLimits(const bool &user = true) {
    std::vector<Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>> ret;
    auto njoints = user ? user_joint_names_.size() : model_.joints.size();
    for (size_t i = 0; i < njoints; i++) ret.push_back(getJointLimit(i, user));
    return ret;
  }

  /**
   * Get the limit of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: limit of the joint with the given index
   */
  inline Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> getJointLimit(
      const size_t &index, const bool &user = true) {
    auto joint_type = getJointType(index, user);
    size_t pinocchio_idx = user ? joint_index_user2pinocchio_[index] : index;
    size_t start_idx = model_.idx_qs[pinocchio_idx], nq = model_.nqs[pinocchio_idx],
           dim_joint = getJointDim(index, user);
    Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret;
    ASSERT(dim_joint == 1, "Only support joint with dim 1 but joint" +
                               getJointNames(user)[index] + " has dim " +
                               std::to_string(dim_joint));
    // std::cout << joint_type << " " << joint_type[joint_prefix.size()] << " " <<
    // joint_type[joint_prefix.size() + 1] << " " <<  nq << " " << dim_joint << " " <<
    // std::endl;
    if (joint_type[joint_prefix_.size()] == 'P' ||
        (joint_type[joint_prefix_.size()] == 'R' &&
         joint_type[joint_prefix_.size() + 1] != 'U')) {
      ret = Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>(nq, 2);
      for (size_t j = 0; j < nq; j++) {
        ret(j, 0) = model_.lowerPositionLimit[start_idx + j];
        ret(j, 1) = model_.upperPositionLimit[start_idx + j];
      }
    } else if (joint_type[joint_prefix_.size()] == 'R' &&
               joint_type[joint_prefix_.size() + 1] == 'U') {
      ret = Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>(1, 2);
      ret(0, 0) = -3.14159265359, ret(0, 1) = 3.14159265359;
    }
    return ret;
  }

  /**
   * Get the id of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: id of the joint with the given index
   */
  inline size_t getJointId(const size_t &index, const bool &user = true) {
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
  inline VectorXI getJointIds(const bool &user = true) {
    if (user)
      return vidx_;
    else {
      auto ret = VectorXI(model_.idx_vs.size());
      for (size_t i = 0; i < model_.idx_vs.size(); i++) ret[i] = model_.idx_vs[i];
      return ret;
    }
  }

  /**
   * Get the dimension of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: dimension of the joint with the given index
   */
  inline size_t getJointDim(const size_t &index, const bool &user = true) {
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
  inline VectorXI getJointDims(const bool &user = true) {
    if (user)
      return nvs_;
    else {
      auto ret = VectorXI(model_.nvs.size());
      for (size_t i = 0; i < model_.nvs.size(); i++) ret[i] = model_.nvs[i];
      return ret;
    }
  }

  /**
   * Get the parent of the joint with the given index.
   *
   * @param index: joint index to query
   * @param user: if ``true``, the joint index follows the order you passed to the
   *    constructor or the default order
   * @return: parent of the joint with the given index
   */
  inline size_t getParent(const size_t &index, const bool &user = true) {
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
  inline VectorXI getParents(const bool &user = true) {
    if (user)
      return parents_;
    else {
      auto ret = VectorXI(model_.parents.size());
      for (size_t i = 0; i < model_.parents.size(); i++) ret[i] = model_.parents[i];
      return ret;
    }
  }

  /**
   * Get the name of all the links.
   *
   * @param user: if ``true``, we get the name of the links in the order you passed
   *    to the constructor or the default order
   * @return: name of the links
   */
  inline std::vector<std::string> getLinkNames(const bool &user = true) {
    if (user)
      return user_link_names_;
    else {
      std::vector<std::string> link_names;
      for (size_t i = 0; i < model_.frames.size(); i++)
        if (model_.frames[i].type == pinocchio::BODY)
          link_names.push_back(model_.frames[i].name);
      return link_names;
    }
  }

  /**
   * Get the name of all the joints. Again, Pinocchio might split a joint into
   * multiple joints.
   *
   * @param user: if ``true``, we get the name of the joints in the order you passed
   *    to the constructor or the default order
   * @return: name of the joints
   */
  inline std::vector<std::string> getJointNames(const bool &user = true) {
    if (user) return user_joint_names_;
    // we need to ignore the "universe" joint
    return std::vector<std::string>(model_.names.begin() + 1, model_.names.end());
  }

  std::vector<std::vector<size_t>> getSupports(const bool &user = true) {
    if (user) {
      std::vector<std::vector<size_t>> ret;
      return ret;
    } else
      return model_.supports;
  }

  std::vector<std::vector<size_t>> getSubtrees(const bool &user = true) {
    if (user) {
      std::vector<std::vector<size_t>> ret;
      return ret;
    } else
      return model_.subtrees;
  }

  /// Frame is a Pinocchio internal data type which is not supported outside this class.
  void printFrames(void);

  int getNFrames(void) { return model_.nframes; }

  /*
      // The original model for debug only
      inline size_t getDimQpos(void) { return model.nv; }

      int getNQ(void) { return model.nq; }

      int getNV(void) { return model.nv; }

      int getNJoints(void) { return model.njoints; }

      int getNBodies(void) { return model.nbodies; }

      std::vector<std::vector<size_t>> getSupports(void) { return model.supports; };

      std::vector<std::vector<size_t>> getSubtrees(void) { return model.subtrees; };

      int getNLinks(void);

      int getNFrames(void) { return model.nframes; }

      std::vector<int> getNQs(void) { return model.nqs; }

      std::vector<int> getNVs(void) { return model.nvs; }

      std::vector<int> getIDXVs(void) { return model.idx_vs; }

      std::vector<int> getIDXQs(void) { return model.idx_qs; }

      std::vector<std::string> getJointNames(void) { return model.names; }

      std::vector<std::string> getFrameNames(void);

      std::vector<std::string> getLinkNames(void);

      std::vector<size_t> getParents(void) { return model.parents; }

      void printFrames(void);
  */

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
   * Pinocchio might have a different link order or it might add additional links.
   *
   * If you do not pass the the list of link names, the default order might not be the
   * one you want.
   *
   * @param names: list of link names in the order you want
   */
  void setLinkOrder(const std::vector<std::string> &names);

  /**
   * Get the joint indices of the joints in the chain from the root to the given link.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @return: joint indices of the joints in the chain
   */
  std::vector<std::size_t> getChainJointIndex(const std::string &end_effector);

  /**
   * Get the joint names of the joints in the chain from the root to the given link.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @return: joint names of the joints in the chain
   */
  std::vector<std::string> getChainJointName(const std::string &end_effector);

  /**
   * Get a random configuration.
   *
   * @return: random joint configuration
   */
  VectorX getRandomConfiguration();

  /**
   * Compute forward kinematics for the given joint configuration.
   *
   * If you want the result you need to call ``get_link_pose()``
   *
   * @param qpos: joint configuration. Needs to be full configuration, not just the
   *    movegroup joints.
   */
  void computeForwardKinematics(const VectorX &qpos);

  /**
   * Get the pose of the given link.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @return: pose of the link [x, y, z, qw, qx, qy, qz]
   */
  Vector7 getLinkPose(const size_t &index);

  Vector7 getJointPose(const size_t &index);  // TODO: not same as sapien

  /**
   * Compute the full jacobian for the given joint configuration.
   *
   * If you want the result you need to call ``get_link_jacobian()``
   *
   * @param qpos: joint configuration. Needs to be full configuration, not just the
   *    movegroup joints.
   */
  void computeFullJacobian(const VectorX &qpos);

  /**
   * Get the jacobian of the given link.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param local: if ``true``, the jacobian is expressed in the local frame of the
   *    link, otherwise it is expressed in the world frame
   * @return: 6 x n jacobian of the link
   */
  Matrix6x getLinkJacobian(const size_t &index, const bool &local = false);

  /**
   * Compute the jacobian of the given link.
   *
   * @param qpos: joint configuration. Needs to be full configuration, not just the
   *    movegroup joints.
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param local: if ``true`` return the jacobian w.r.t. the instantaneous local frame
   *    of the link
   * @return: 6 x n jacobian of the link
   */
  Matrix6x computeSingleLinkJacobian(const VectorX &qpos, const size_t &index,
                                     bool local = false);

  /**
   * Compute the inverse kinematics using close loop inverse kinematics.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param pose: desired pose of the link [x, y, z, qw, qx, qy, qz]
   * @param q_init: initial joint configuration
   * @param mask: if the value at a given index is ``true``, the joint is *not* used in
   *    the IK
   * @param eps: tolerance for the IK
   * @param maxIter: maximum number of iterations
   * @param dt: time step for the CLIK
   * @param damp: damping for the CLIK
   * @return: joint configuration
   */
  std::tuple<VectorX, bool, Vector6> computeIKCLIK(
      const size_t &index, const Vector7 &pose, const VectorX &q_init,
      const std::vector<bool> &mask, const double &eps = 1e-5,
      const int &maxIter = 1000, const double &dt = 1e-1, const double &damp = 1e-12);

  /**
   * The same as ``compute_IK_CLIK()`` but with it clamps the joint configuration to the
   * given limits.
   *
   * @param index: index of the link (in the order you passed to the constructor or the
   *    default order)
   * @param pose: desired pose of the link [x, y, z, qw, qx, qy, qz]
   * @param q_init: initial joint configuration
   * @param q_min: minimum joint configuration
   * @param q_max: maximum joint configuration
   * @param eps: tolerance for the IK
   * @param maxIter: maximum number of iterations
   * @param dt: time step for the CLIK
   * @param damp: damping for the CLIK
   * @return: joint configuration
   */
  std::tuple<VectorX, bool, Vector6> computeIKCLIKJL(
      const size_t &index, const Vector7 &pose, const VectorX &q_init,
      const VectorX &qmin, const VectorX &qmax, const double &eps = 1e-5,
      const int &maxIter = 1000, const double &dt = 1e-1, const double &damp = 1e-12);
};

template <typename T>
using PinocchioModelTpl_ptr = std::shared_ptr<PinocchioModelTpl<T>>;

using PinocchioModeld = PinocchioModelTpl<double>;
using PinocchioModelf = PinocchioModelTpl<float>;
using PinocchioModeld_ptr = PinocchioModelTpl_ptr<double>;
using PinocchioModelf_ptr = PinocchioModelTpl_ptr<float>;
