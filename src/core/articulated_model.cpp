#include "mplib/core/articulated_model.h"

#include <algorithm>
#include <memory>

#include "mplib/macros/assert.h"
#include "mplib/utils/color_printing.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_ARTICULATED_MODEL(S) template class ArticulatedModelTpl<S>

DEFINE_TEMPLATE_ARTICULATED_MODEL(float);
DEFINE_TEMPLATE_ARTICULATED_MODEL(double);

template <typename S>
ArticulatedModelTpl<S>::ArticulatedModelTpl(const std::string &urdf_filename,
                                            const std::string &srdf_filename,
                                            const std::string_view name,
                                            const Vector3<S> &gravity,
                                            const std::vector<std::string> &link_names,
                                            const std::vector<std::string> &joint_names,
                                            bool convex, bool verbose)
    : pinocchio_model_(std::make_shared<kinematics::PinocchioModelTpl<S>>(
          urdf_filename, gravity, verbose)),
      fcl_model_(std::make_shared<collision_detection::FCLModelTpl<S>>(
          urdf_filename, convex, verbose)),
      verbose_(verbose) {
  name_ = name.data() ? name : fcl_model_->getName();
  fcl_model_->setName(name_);
  user_link_names_ =
      link_names.size() == 0 ? pinocchio_model_->getLinkNames(false) : link_names;
  user_joint_names_ =
      joint_names.size() == 0 ? pinocchio_model_->getJointNames(false) : joint_names;
  pinocchio_model_->setLinkOrder(user_link_names_);
  pinocchio_model_->setJointOrder(user_joint_names_);
  fcl_model_->setLinkOrder(user_link_names_);
  fcl_model_->removeCollisionPairsFromSRDF(srdf_filename);
  current_qpos_ = VectorX<S>::Constant(pinocchio_model_->getModel().nv, 0);
  setMoveGroup(user_link_names_);
  setBasePose(Pose<S>());  // initialize base pose to identity
}

template <typename S>
std::unique_ptr<ArticulatedModelTpl<S>> ArticulatedModelTpl<S>::createFromURDFString(
    const std::string &urdf_string, const std::string &srdf_string,
    const std::vector<collision_detection::FCLObjectPtr<S>> &collision_links,
    const std::string_view name, const Vector3<S> &gravity,
    const std::vector<std::string> &link_names,
    const std::vector<std::string> &joint_names, bool verbose) {
  auto articulation = std::make_unique<ArticulatedModelTpl<S>>(Secret());

  auto pinocchio_model = articulation->pinocchio_model_ =
      kinematics::PinocchioModelTpl<S>::createFromURDFString(urdf_string, gravity,
                                                             verbose);
  auto fcl_model = articulation->fcl_model_ =
      collision_detection::FCLModelTpl<S>::createFromURDFString(
          urdf_string, collision_links, verbose);
  articulation->name_ = name.data() ? name : fcl_model->getName();
  fcl_model->setName(articulation->name_);
  articulation->verbose_ = verbose;

  auto user_link_names = articulation->user_link_names_ =
      link_names.size() == 0 ? pinocchio_model->getLinkNames(false) : link_names;
  auto user_joint_names = articulation->user_joint_names_ =
      joint_names.size() == 0 ? pinocchio_model->getJointNames(false) : joint_names;
  pinocchio_model->setLinkOrder(user_link_names);
  pinocchio_model->setJointOrder(user_joint_names);
  fcl_model->setLinkOrder(user_link_names);
  fcl_model->removeCollisionPairsFromSRDFString(srdf_string);
  articulation->current_qpos_ = VectorX<S>::Constant(pinocchio_model->getModel().nv, 0);
  articulation->setMoveGroup(user_link_names);

  return articulation;
}

template <typename S>
std::vector<std::string> ArticulatedModelTpl<S>::getMoveGroupJointNames() const {
  std::vector<std::string> ret;
  for (const auto &i : move_group_user_joints_) ret.push_back(user_joint_names_[i]);
  return ret;
}

template <typename S>
void ArticulatedModelTpl<S>::setMoveGroup(const std::string &end_effector) {
  std::vector<std::string> end_effectors {end_effector};
  setMoveGroup(end_effectors);
}

template <typename S>
void ArticulatedModelTpl<S>::setMoveGroup(
    const std::vector<std::string> &end_effectors) {
  move_group_end_effectors_ = end_effectors;
  move_group_user_joints_ = {};
  for (const auto &end_effector : end_effectors) {
    auto joint_i = pinocchio_model_->getChainJointIndex(end_effector);
    move_group_user_joints_.insert(move_group_user_joints_.begin(), joint_i.begin(),
                                   joint_i.end());
  }
  std::sort(move_group_user_joints_.begin(), move_group_user_joints_.end());
  auto end_unique =
      std::unique(move_group_user_joints_.begin(), move_group_user_joints_.end());
  move_group_user_joints_.erase(end_unique, move_group_user_joints_.end());
  move_group_qpos_dim_ = 0;
  for (const auto &i : move_group_user_joints_)
    move_group_qpos_dim_ += pinocchio_model_->getJointDim(i);
}

template <typename S>
void ArticulatedModelTpl<S>::setQpos(const VectorX<S> &qpos, bool full) {
  if (full) {
    ASSERT(
        static_cast<size_t>(qpos.size()) == static_cast<size_t>(current_qpos_.size()),
        "Length of provided qpos " + std::to_string(qpos.size()) +
            " =/= dimension of full qpos: " + std::to_string(current_qpos_.size()));
    current_qpos_ = qpos;
  } else {
    ASSERT(static_cast<size_t>(qpos.size()) == move_group_qpos_dim_,
           "Length of provided qpos " + std::to_string(qpos.size()) +
               " =/= dimension of move_group qpos: " +
               std::to_string(move_group_qpos_dim_));
    size_t len = 0;
    for (const auto &i : move_group_user_joints_) {
      auto start_idx = pinocchio_model_->getJointId(i),
           dim_i = pinocchio_model_->getJointDim(i);
      for (size_t j = 0; j < dim_i; j++) current_qpos_[start_idx + j] = qpos[len++];
    }
  }
  pinocchio_model_->computeForwardKinematics(current_qpos_);
  if (verbose_) print_verbose("current_qpos ", current_qpos_);
  std::vector<Pose<S>> link_pose;
  for (size_t i = 0; i < user_link_names_.size(); i++)
    link_pose.push_back(Pose<S>(base_pose_ * pinocchio_model_->getLinkPose(i)));
  fcl_model_->updateCollisionObjects(link_pose);
}

}  // namespace mplib
