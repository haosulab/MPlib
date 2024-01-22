#include "mplib/articulated_model.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "mplib/urdf_utils.h"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf/geometry.hxx"
#include "pinocchio/parsers/urdf/model.hxx"
#include "pinocchio/parsers/utils.hpp"

#define DEFINE_TEMPLATE_AM(DATATYPE) template class ArticulatedModelTpl<DATATYPE>;

DEFINE_TEMPLATE_AM(float)

DEFINE_TEMPLATE_AM(double)

template <typename DATATYPE>
ArticulatedModelTpl<DATATYPE>::ArticulatedModelTpl(
    const std::string &urdf_filename, const std::string &srdf_filename,
    const Vector3 &gravity, const std::vector<std::string> &joint_names,
    const std::vector<std::string> &link_names, const bool &verbose, const bool &convex)
    : pinocchio_model_(urdf_filename, gravity, verbose),
      fcl_model_(urdf_filename, verbose, convex),
      verbose_(verbose) {
  user_link_names_ =
      link_names.size() == 0 ? pinocchio_model_.getLinkNames(false) : link_names;
  user_joint_names_ =
      joint_names.size() == 0 ? pinocchio_model_.getJointNames(false) : joint_names;
  pinocchio_model_.setLinkOrder(user_link_names_);
  pinocchio_model_.setJointOrder(user_joint_names_);
  fcl_model_.setLinkOrder(user_link_names_);
  fcl_model_.removeCollisionPairsFromSrdf(srdf_filename);
  current_qpos_ = VectorX::Constant(pinocchio_model_.getModel().nv, 0);
  setMoveGroup(user_link_names_);
  setBasePose({0, 0, 0, 1, 0, 0, 0});  // initialize base pose to identity
}

template <typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setMoveGroup(const std::string &end_effector) {
  std::vector<std::string> end_effectors = {end_effector};
  setMoveGroup(end_effectors);
}

template <typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setMoveGroup(
    const std::vector<std::string> &end_effectors) {
  move_group_end_effectors_ = end_effectors;
  move_group_user_joints_ = {};
  for (auto end_effector : end_effectors) {
    auto joint_i = pinocchio_model_.getChainJointIndex(end_effector);
    move_group_user_joints_.insert(move_group_user_joints_.begin(), joint_i.begin(),
                                   joint_i.end());
  }
  std::sort(move_group_user_joints_.begin(), move_group_user_joints_.end());
  auto end_unique =
      std::unique(move_group_user_joints_.begin(), move_group_user_joints_.end());
  move_group_user_joints_.erase(end_unique, move_group_user_joints_.end());
  move_group_qpos_dim_ = 0;
  for (auto i : move_group_user_joints_)
    move_group_qpos_dim_ += pinocchio_model_.getJointDim(i);
}

template <typename DATATYPE>
std::vector<std::string> ArticulatedModelTpl<DATATYPE>::getMoveGroupJointName(void) {
  std::vector<std::string> ret;
  for (auto i : move_group_user_joints_) ret.push_back(user_joint_names_[i]);
  return ret;
}

template <typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setQpos(const VectorX &qpos, const bool &full) {
  if (full)
    current_qpos_ = qpos;
  else {
    ASSERT(static_cast<size_t>(qpos.size()) == move_group_qpos_dim_,
           "Length of provided qpos " + std::to_string(qpos.size()) +
               " =/= dimension of move_group qpos: " +
               std::to_string(move_group_qpos_dim_));
    size_t len = 0;
    for (auto i : move_group_user_joints_) {
      auto start_idx = pinocchio_model_.getJointId(i),
           dim_i = pinocchio_model_.getJointDim(i);
      for (size_t j = 0; j < dim_i; j++) current_qpos_[start_idx + j] = qpos[len++];
    }
  }
  pinocchio_model_.computeForwardKinematics(current_qpos_);
  if (verbose_) print_verbose("current_qpos ", current_qpos_);
  std::vector<Transform3> link_pose;
  for (size_t i = 0; i < user_link_names_.size(); i++) {
    Vector7 pose_i = pinocchio_model_.getLinkPose(i);
    Transform3 tmp_i;
    tmp_i.linear() = Quaternion(pose_i[3], pose_i[4], pose_i[5], pose_i[6]).matrix();
    tmp_i.translation() = pose_i.head(3);
    tmp_i = base_tf_ * tmp_i;  // base_tf is the pose of the robot base
    link_pose.push_back(tmp_i);
  }
  fcl_model_.updateCollisionObjects(link_pose);
}

template <typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setBasePose(const Vector7 &pose) {
  base_pose_ = pose;
  base_tf_.translation() = pose.head(3);
  base_tf_.linear() = Eigen::Quaternion<DATATYPE>(pose[3], pose[4], pose[5], pose[6])
                          .toRotationMatrix();
  setQpos(
      current_qpos_,
      true);  // we don't need to update Qpos, but this also updates fcl, which we need
}
