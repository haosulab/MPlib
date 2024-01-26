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
    : pinocchio_model(urdf_filename, gravity, verbose),
      fcl_model(urdf_filename, verbose, convex),
      verbose(verbose) {
  user_link_names =
      link_names.size() == 0 ? pinocchio_model.getLinkNames(false) : link_names;
  user_joint_names =
      joint_names.size() == 0 ? pinocchio_model.getJointNames(false) : joint_names;
  pinocchio_model.setLinkOrder(user_link_names);
  pinocchio_model.setJointOrder(user_joint_names);
  fcl_model.setLinkOrder(user_link_names);
  fcl_model.removeCollisionPairsFromSrdf(srdf_filename);
  current_qpos = VectorX::Constant(pinocchio_model.getModel().nv, 0);
  setMoveGroup(user_link_names);
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
  move_group_end_effectors = end_effectors;
  move_group_user_joints = {};
  for (auto end_effector : end_effectors) {
    auto joint_i = pinocchio_model.getChainJointIndex(end_effector);
    move_group_user_joints.insert(move_group_user_joints.begin(), joint_i.begin(),
                                  joint_i.end());
  }
  std::sort(move_group_user_joints.begin(), move_group_user_joints.end());
  auto end_unique =
      std::unique(move_group_user_joints.begin(), move_group_user_joints.end());
  move_group_user_joints.erase(end_unique, move_group_user_joints.end());
  move_group_qpos_dim = 0;
  for (auto i : move_group_user_joints)
    move_group_qpos_dim += pinocchio_model.getJointDim(i);
}

template <typename DATATYPE>
std::vector<std::string> ArticulatedModelTpl<DATATYPE>::getMoveGroupJointName(void) {
  std::vector<std::string> ret;
  for (auto i : move_group_user_joints) ret.push_back(user_joint_names[i]);
  return ret;
}

template <typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setQpos(const VectorX &qpos, const bool &full) {
  if (full) {
    ASSERT(static_cast<size_t>(qpos.size()) == static_cast<size_t>(current_qpos.size()),
           "Length of provided qpos " + std::to_string(qpos.size()) +
               " =/= dimension of full qpos: " +
               std::to_string(current_qpos.size()));
    current_qpos = qpos;
  } else {
    ASSERT(static_cast<size_t>(qpos.size()) == move_group_qpos_dim,
           "Length of provided qpos " + std::to_string(qpos.size()) +
               " =/= dimension of move_group qpos: " +
               std::to_string(move_group_qpos_dim));
    size_t len = 0;
    for (auto i : move_group_user_joints) {
      auto start_idx = pinocchio_model.getJointId(i),
           dim_i = pinocchio_model.getJointDim(i);
      for (size_t j = 0; j < dim_i; j++) current_qpos[start_idx + j] = qpos[len++];
    }
  }
  pinocchio_model.computeForwardKinematics(current_qpos);
  if (verbose) print_verbose("current_qpos ", current_qpos);
  std::vector<Transform3> link_pose;
  for (size_t i = 0; i < user_link_names.size(); i++) {
    Vector7 pose_i = pinocchio_model.getLinkPose(i);
    Transform3 tmp_i;
    tmp_i.linear() = Quaternion(pose_i[3], pose_i[4], pose_i[5], pose_i[6]).matrix();
    tmp_i.translation() = pose_i.head(3);
    tmp_i = base_tf * tmp_i;  // base_tf is the pose of the robot base
    link_pose.push_back(tmp_i);
  }
  fcl_model.updateCollisionObjects(link_pose);
}

template <typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setBasePose(const Vector7 &pose) {
  base_pose = pose;
  base_tf.translation() = pose.head(3);
  base_tf.linear() = Eigen::Quaternion<DATATYPE>(pose[3], pose[4], pose[5], pose[6])
                         .toRotationMatrix();
  setQpos(
      current_qpos,
      true);  // we don't need to update Qpos, but this also updates fcl, which we need
}
