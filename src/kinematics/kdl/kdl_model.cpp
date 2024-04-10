#include "mplib/kinematics/kdl/kdl_model.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf_world/types.h>

#include "mplib/kinematics/kdl/kdl_utils.h"
#include "mplib/macros/assert.h"

namespace mplib::kinematics::kdl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_KDL_MODEL(S) template class KDLModelTpl<S>

DEFINE_TEMPLATE_KDL_MODEL(float);
DEFINE_TEMPLATE_KDL_MODEL(double);

template <typename S>
KDLModelTpl<S>::KDLModelTpl(const std::string &urdf_filename,
                            const std::vector<std::string> &link_names,
                            const std::vector<std::string> &joint_names, bool verbose)
    : user_link_names_(link_names), user_joint_names_(joint_names), verbose_(verbose) {
  for (size_t i = 0; i < joint_names.size(); i++)
    user_joint_idx_mapping_[joint_names[i]] = i;
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_filename);
  treeFromUrdfModel(urdf_model, tree_, tree_root_name_, verbose);

  KDL::SegmentMap segment_map = tree_.getSegments();
  joint_mapping_kdl_2_user_.resize(tree_.getNrOfJoints());

  for (const auto &[key, element] : segment_map) {
    std::string joint_name = element.segment.getJoint().getName();
    if (user_joint_idx_mapping_.find(joint_name) != user_joint_idx_mapping_.end())
      joint_mapping_kdl_2_user_[element.q_nr] = user_joint_idx_mapping_[joint_name];
  }
}

template <typename S>
std::tuple<VectorX<S>, int> KDLModelTpl<S>::chainIKLMA(size_t index,
                                                       const VectorX<S> &q0,
                                                       const Pose<S> &pose) const {
  KDL::Chain chain;
  Vector6<double> L {1, 1, 1, 0.01, 0.01, 0.01};
  ASSERT(index < user_link_names_.size(), "link index out of bound");
  tree_.getChain(tree_root_name_, user_link_names_[index], chain);

  KDL::Frame frame_goal = KDL::Frame(
      KDL::Rotation::Quaternion(pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w()),
      KDL::Vector(pose.p(0), pose.p(1), pose.p(2)));
  KDL::ChainIkSolverPos_LMA solver(chain, L);
  int n = chain.getNrOfJoints();
  KDL::JntArray q_init(n);
  KDL::JntArray q_sol(n);
  std::vector<int> idx;
  for (const auto &seg : chain.segments) {
    auto joint = seg.getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
      idx.push_back(user_joint_idx_mapping_.at(joint.getName()));
  }
  for (int i = 0; i < n; i++) q_init(i) = q0[idx[i]];
  auto retval = solver.CartToJnt(q_init, frame_goal, q_sol);
  VectorX<S> q1 = q0;
  for (int i = 0; i < n; i++) q1[idx[i]] = q_sol(i);
  return {q1, retval};
}

template <typename S>
std::tuple<VectorX<S>, int> KDLModelTpl<S>::chainIKNR(size_t index,
                                                      const VectorX<S> &q0,
                                                      const Pose<S> &pose) const {
  KDL::Chain chain;
  ASSERT(index < user_link_names_.size(), "link index out of bound");
  tree_.getChain(tree_root_name_, user_link_names_[index], chain);

  KDL::Frame frame_goal = KDL::Frame(
      KDL::Rotation::Quaternion(pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w()),
      KDL::Vector(pose.p(0), pose.p(1), pose.p(2)));

  KDL::ChainFkSolverPos_recursive fkpossolver(chain);
  KDL::ChainIkSolverVel_pinv ikvelsolver(chain);

  KDL::ChainIkSolverPos_NR solver(chain, fkpossolver, ikvelsolver);

  int n = chain.getNrOfJoints();
  KDL::JntArray q_init(n);
  KDL::JntArray q_sol(n);
  std::vector<int> idx;
  for (const auto &seg : chain.segments) {
    auto joint = seg.getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
      idx.push_back(user_joint_idx_mapping_.at(joint.getName()));
  }
  for (int i = 0; i < n; i++) q_init(i) = q0[idx[i]];
  auto retval = solver.CartToJnt(q_init, frame_goal, q_sol);
  VectorX<S> q1 = q0;
  for (int i = 0; i < n; i++) q1[idx[i]] = q_sol(i);
  return {q1, retval};
}

template <typename S>
std::tuple<VectorX<S>, int> KDLModelTpl<S>::chainIKNRJL(size_t index,
                                                        const VectorX<S> &q0,
                                                        const Pose<S> &pose,
                                                        const VectorX<S> &qmin,
                                                        const VectorX<S> &qmax) const {
  KDL::Chain chain;
  ASSERT(index < user_link_names_.size(), "link index out of bound");
  tree_.getChain(tree_root_name_, user_link_names_[index], chain);

  KDL::Frame frame_goal = KDL::Frame(
      KDL::Rotation::Quaternion(pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w()),
      KDL::Vector(pose.p(0), pose.p(1), pose.p(2)));

  KDL::ChainFkSolverPos_recursive fkpossolver(chain);
  KDL::ChainIkSolverVel_pinv ikvelsolver(chain);
  KDL::JntArray q_min(chain.getNrOfJoints()), q_max(chain.getNrOfJoints());
  int n = chain.getNrOfJoints();
  KDL::JntArray q_init(n);
  KDL::JntArray q_sol(n);
  std::vector<int> idx;
  for (const auto &seg : chain.segments) {
    auto joint = seg.getJoint();
    if (joint.getType() != KDL::Joint::Fixed)
      idx.push_back(user_joint_idx_mapping_.at(joint.getName()));
  }

  for (int i = 0; i < n; i++) {
    q_min(i) = qmin[idx[i]];
    q_max(i) = qmax[idx[i]];
  }

  KDL::ChainIkSolverPos_NR_JL solver(chain, q_min, q_max, fkpossolver, ikvelsolver);

  for (int i = 0; i < n; i++) q_init(i) = q0[idx[i]];
  auto retval = solver.CartToJnt(q_init, frame_goal, q_sol);
  VectorX<S> q1 = q0;
  for (int i = 0; i < n; i++) q1[idx[i]] = q_sol(i);
  return {q1, retval};
}

template <typename S>
std::tuple<VectorX<S>, int> KDLModelTpl<S>::TreeIKNRJL(
    const std::vector<std::string> endpoints, const VectorX<S> &q0,
    const std::vector<Pose<S>> &poses, const VectorX<S> &qmin,
    const VectorX<S> &qmax) const {
  KDL::TreeFkSolverPos_recursive fkpossolver(tree_);
  KDL::TreeIkSolverVel_wdls ikvelsolver(tree_, endpoints);
  ikvelsolver.setLambda(1e-6);

  int n = tree_.getNrOfJoints();
  KDL::JntArray q_min(n), q_max(n), q_init(n), q_sol(n);

  for (int i = 0; i < n; i++) {
    q_min(i) = qmin[joint_mapping_kdl_2_user_[i]];
    q_max(i) = qmax[joint_mapping_kdl_2_user_[i]];
  }

  KDL::Frames frames;
  for (size_t i = 0; i < endpoints.size(); i++)
    frames[endpoints[i]] =
        KDL::Frame(KDL::Rotation::Quaternion(poses[i].q.x(), poses[i].q.y(),
                                             poses[i].q.z(), poses[i].q.w()),
                   KDL::Vector(poses[i].p(0), poses[i].p(1), poses[i].p(2)));

  for (int i = 0; i < n; i++) q_init(i) = q0[joint_mapping_kdl_2_user_[i]];

  KDL::TreeIkSolverPos_NR_JL solver(tree_, endpoints, q_min, q_max, fkpossolver,
                                    ikvelsolver, 1000, 1e-6);

  auto retval = solver.CartToJnt(q_init, frames, q_sol);

  VectorX<S> q1 = q0;
  for (int i = 0; i < n; i++) q1[joint_mapping_kdl_2_user_[i]] = q_sol(i);

  return {q1, retval};
}

}  // namespace mplib::kinematics::kdl
