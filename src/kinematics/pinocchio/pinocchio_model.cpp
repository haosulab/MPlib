#include "mplib/kinematics/pinocchio/pinocchio_model.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <urdf_parser/urdf_parser.h>

#include "mplib/macros/assert.h"
#include "mplib/utils/color_printing.h"
#include "mplib/utils/conversion.h"

namespace mplib::kinematics::pinocchio {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_PINOCCHIO_MODEL(S) template class PinocchioModelTpl<S>

DEFINE_TEMPLATE_PINOCCHIO_MODEL(float);
DEFINE_TEMPLATE_PINOCCHIO_MODEL(double);

template <typename S>
PinocchioModelTpl<S>::PinocchioModelTpl(const std::string &urdf_filename,
                                        const Vector3<S> &gravity, bool verbose)
    : verbose_(verbose) {
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_filename);
  init(urdf_model, gravity);
}

template <typename S>
PinocchioModelTpl<S>::PinocchioModelTpl(const urdf::ModelInterfaceSharedPtr &urdf_model,
                                        const Vector3<S> &gravity, bool verbose)
    : verbose_(verbose) {
  init(urdf_model, gravity);
}

template <typename S>
std::unique_ptr<PinocchioModelTpl<S>> PinocchioModelTpl<S>::createFromURDFString(
    const std::string &urdf_string, const Vector3<S> &gravity, bool verbose) {
  auto urdf = urdf::parseURDF(urdf_string);
  return std::make_unique<PinocchioModelTpl<S>>(urdf, gravity, verbose);
}

template <typename S>
void PinocchioModelTpl<S>::init(const urdf::ModelInterfaceSharedPtr &urdf_model,
                                const Vector3<S> &gravity) {
  urdf_model_ = urdf_model;

  pinocchio::urdf::details::UrdfVisitor<S, 0, pinocchio::JointCollectionDefaultTpl>
      visitor(model_);
  if (verbose_) {
    print_verbose("Begin to parse URDF");
    visitor.log = &std::cout;
  }
  if (not urdf_model_)
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");

  visitor.setName(urdf_model_->getName());
  const urdf::LinkConstSharedPtr root_link = urdf_model_->getRoot();
  visitor.addRootJoint(convertInertial<S>(root_link->inertial), root_link->name);
  for (const auto &child : root_link->child_links) dfsParseTree(child, visitor);

  model_.gravity = {gravity, Vector3<S> {0, 0, 0}};
  data_ = pinocchio::DataTpl<S>(model_);
  if (verbose_) print_verbose("Begin to set link order and joint order");
  setLinkOrder(getLinkNames(false));
  setJointOrder(getJointNames(false));
}

template <typename S>
void PinocchioModelTpl<S>::dfsParseTree(const urdf::LinkConstSharedPtr &link,
                                        pinocchio::UrdfVisitorBase<S> &visitor) {
  const urdf::JointConstSharedPtr joint =
      urdf::const_pointer_cast<urdf::Joint>(link->parent_joint);
  if (verbose_) print_verbose("Joint ", joint->name, " ", joint.get());
  if (joint) {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(link->getParent() && link);

    const std::string &joint_name = joint->name;
    const std::string &link_name = link->name;
    const std::string &parent_link_name = link->getParent()->name;
    const pinocchio::FrameIndex parent_frame_id = visitor.getBodyId(parent_link_name);

    // Transformation from the parent link to the joint origin
    const pinocchio::SE3Tpl<S> joint_placement =
        toSE3<S>(joint->parent_to_joint_origin_transform);
    const pinocchio::InertiaTpl<S> Y = convertInertial<S>(link->inertial);

    const S infty = std::numeric_limits<S>::infinity();
    VectorX<S> max_effort(1), max_velocity(1), min_config(1), max_config(1);
    VectorX<S> friction(VectorX<S>::Constant(1, 0.)),
        damping(VectorX<S>::Constant(1, 0.));
    const Vector3<S> axis(joint->axis.x, joint->axis.y, joint->axis.z);

    std::ostringstream joint_info;

    switch (joint->type) {
      case urdf::Joint::FLOATING:
        joint_info << "joint FreeFlyer";

        max_effort = VectorX<S>::Constant(6, infty);
        max_velocity = VectorX<S>::Constant(6, infty);
        min_config = VectorX<S>::Constant(7, -infty);
        max_config = VectorX<S>::Constant(7, infty);
        min_config.tail(4).setConstant(-1.01);
        max_config.tail(4).setConstant(1.01);

        friction = VectorX<S>::Constant(6, 0.);
        damping = VectorX<S>::Constant(6, 0.);

        visitor.addJointAndBody(pinocchio::UrdfVisitorBase<S>::FLOATING, axis,
                                parent_frame_id, joint_placement, joint->name, Y,
                                link->name, max_effort, max_velocity, min_config,
                                max_config, friction, damping);
        break;

      case ::urdf::Joint::REVOLUTE:
        joint_info << "joint REVOLUTE with axis";

        // TODO: I think the URDF standard forbids REVOLUTE with no limits.
        ASSERT(joint->limits, "REVOLUTE without limits");
        if (joint->limits) {
          max_effort << joint->limits->effort;
          max_velocity << joint->limits->velocity;
          min_config << joint->limits->lower;
          max_config << joint->limits->upper;
        }

        if (joint->dynamics) {
          friction << joint->dynamics->friction;
          damping << joint->dynamics->damping;
        }

        visitor.addJointAndBody(pinocchio::UrdfVisitorBase<S>::REVOLUTE, axis,
                                parent_frame_id, joint_placement, joint->name, Y,
                                link->name, max_effort, max_velocity, min_config,
                                max_config, friction, damping);
        break;

      case ::urdf::Joint::CONTINUOUS:  // Revolute joint with no joint limits
        joint_info << "joint CONTINUOUS with axis";

        min_config.resize(2);
        max_config.resize(2);
        min_config << -1.01, -1.01;
        max_config << 1.01, 1.01;

        if (joint->limits) {
          max_effort << joint->limits->effort;
          max_velocity << joint->limits->velocity;
        } else {
          max_effort << infty;
          max_velocity << infty;
        }

        if (joint->dynamics) {
          friction << joint->dynamics->friction;
          damping << joint->dynamics->damping;
        }

        visitor.addJointAndBody(pinocchio::UrdfVisitorBase<S>::CONTINUOUS, axis,
                                parent_frame_id, joint_placement, joint->name, Y,
                                link->name, max_effort, max_velocity, min_config,
                                max_config, friction, damping);
        break;

      case ::urdf::Joint::PRISMATIC:
        joint_info << "joint PRISMATIC with axis";

        // TODO: I think the URDF standard forbids REVOLUTE with no limits.
        ASSERT(joint->limits, "PRISMATIC without limits");
        if (joint->limits) {
          max_effort << joint->limits->effort;
          max_velocity << joint->limits->velocity;
          min_config << joint->limits->lower;
          max_config << joint->limits->upper;
        }

        if (joint->dynamics) {
          friction << joint->dynamics->friction;
          damping << joint->dynamics->damping;
        }

        visitor.addJointAndBody(pinocchio::UrdfVisitorBase<S>::PRISMATIC, axis,
                                parent_frame_id, joint_placement, joint->name, Y,
                                link->name, max_effort, max_velocity, min_config,
                                max_config, friction, damping);
        break;

      case ::urdf::Joint::PLANAR:
        joint_info << "joint PLANAR with normal axis along Z";

        max_effort = VectorX<S>::Constant(3, infty);
        max_velocity = VectorX<S>::Constant(3, infty);
        min_config = VectorX<S>::Constant(4, -infty);
        max_config = VectorX<S>::Constant(4, infty);

        min_config.tail(2).setConstant(-1.01);
        max_config.tail(2).setConstant(1.01);

        friction = VectorX<S>::Constant(3, 0.);
        damping = VectorX<S>::Constant(3, 0.);

        visitor.addJointAndBody(pinocchio::UrdfVisitorBase<S>::PLANAR, axis,
                                parent_frame_id, joint_placement, joint->name, Y,
                                link->name, max_effort, max_velocity, min_config,
                                max_config, friction, damping);
        break;

      case ::urdf::Joint::FIXED:
        joint_info << "fixed joint";
        visitor.addFixedJointAndBody(parent_frame_id, joint_placement, joint_name, Y,
                                     link_name);
        break;

      default:
        throw std::invalid_argument("The type of joint " + joint_name +
                                    " is not supported.");
    }

    visitor.getBodyId(link->name);
    visitor << "Adding Body" << '\n'
            << '\"' << link_name << "\" connected to \"" << parent_link_name
            << "\" through joint \"" << joint_name << "\"\n"
            << "joint type: " << joint_info.str() << '\n'
            << "joint placement:\n"
            << joint_placement << '\n'
            << "body info: " << '\n'
            << "  mass: " << Y.mass() << '\n'
            << "  lever: " << Y.lever().transpose() << '\n'
            << "  inertia elements (Ixx,Iyx,Iyy,Izx,Izy,Izz): "
            << Y.inertia().data().transpose() << '\n'
            << '\n';
  } else if (link->getParent())
    throw std::invalid_argument(link->name + " - joint information missing.");
  for (const auto &child : link->child_links) dfsParseTree(child, visitor);
  if (link->child_links.empty()) leaf_links_.push_back(link->name);
}

template <typename S>
std::set<std::pair<std::string, std::string>> PinocchioModelTpl<S>::getAdjacentLinks()
    const {
  std::set<std::pair<std::string, std::string>> ret;

  const auto root_name = urdf_model_->getRoot()->name;
  for (auto link_name : getLeafLinks())
    while (link_name != root_name) {
      const auto parent_link = urdf_model_->getLink(link_name)->getParent();
      const auto parent_name = parent_link->name;
      ret.insert({parent_name, link_name});
      // if the parent has a fixed link to its parent, also disable collision
      if (parent_name != root_name &&
          parent_link->parent_joint->type == urdf::Joint::FIXED)
        ret.insert({parent_link->getParent()->name, link_name});
      link_name = parent_name;
    }
  return ret;
}

template <typename S>
void PinocchioModelTpl<S>::setLinkOrder(const std::vector<std::string> &names) {
  user_link_names_ = names;
  link_index_user2pinocchio_ = VectorXi(names.size());
  for (size_t i = 0; i < names.size(); i++) {
    const auto pinocchio_idx = model_.getFrameId(names[i], pinocchio::BODY);
    if (pinocchio_idx == static_cast<size_t>(model_.nframes))
      throw std::invalid_argument(names[i] + " is a invalid names in setLinkOrder");
    link_index_user2pinocchio_[i] = pinocchio_idx;
  }
}

template <typename S>
void PinocchioModelTpl<S>::setJointOrder(const std::vector<std::string> &names) {
  user_joint_names_ = names;
  v_index_user2pinocchio_ = VectorXi(model_.nv);
  vidx_ = VectorXi(names.size());
  qidx_ = VectorXi(names.size());
  nvs_ = VectorXi(names.size());
  nqs_ = VectorXi(names.size());
  parents_ = VectorXi(names.size());
  joint_index_user2pinocchio_ = VectorXi(names.size());
  joint_index_pinocchio2user_ = VectorXi::Constant(model_.njoints, -1);

  size_t len_v = 0, len_q = 0;
  for (size_t i = 0; i < names.size(); i++) {
    const auto pinocchio_idx = model_.getJointId(names[i]);
    if (pinocchio_idx == static_cast<size_t>(model_.njoints))
      throw std::invalid_argument(
          names[i] + " is either an invalid or unsupported joint in your URDF");
    vidx_[i] = len_v;
    qidx_[i] = len_q;
    nvs_[i] = model_.nvs[pinocchio_idx];
    nqs_[i] = model_.nqs[pinocchio_idx];
    parents_[i] = model_.parents[pinocchio_idx];
    joint_index_user2pinocchio_[i] = pinocchio_idx;
    joint_index_pinocchio2user_[pinocchio_idx] = i;
    len_q += model_.nqs[pinocchio_idx];
    for (int j = 0; j < model_.nvs[pinocchio_idx]; j++)
      v_index_user2pinocchio_[len_v++] = model_.idx_vs[pinocchio_idx] + j;
  }
  ASSERT(len_v == static_cast<size_t>(model_.nv), "setJointOrder failed");
  v_map_user2pinocchio_ = PermutationMatrixX(v_index_user2pinocchio_);
}

template <typename S>
std::vector<std::string> PinocchioModelTpl<S>::getLinkNames(bool user) const {
  if (user)
    return user_link_names_;
  else {
    std::vector<std::string> link_names;
    for (const auto &frame : model_.frames)
      if (frame.type == pinocchio::BODY) link_names.push_back(frame.name);
    return link_names;
  }
}

template <typename S>
std::vector<std::string> PinocchioModelTpl<S>::getJointNames(bool user) const {
  if (user) return user_joint_names_;
  // we need to ignore the "universe" joint
  return std::vector<std::string>(model_.names.begin() + 1, model_.names.end());
}

template <typename S>
VectorXi PinocchioModelTpl<S>::getJointIds(bool user) const {
  if (user)
    return vidx_;
  else {
    auto ret = VectorXi(model_.idx_vs.size());
    for (size_t i = 0; i < model_.idx_vs.size(); i++) ret[i] = model_.idx_vs[i];
    return ret;
  }
}

template <typename S>
std::string PinocchioModelTpl<S>::getJointType(size_t index, bool user) const {
  if (user)
    return model_.joints[joint_index_user2pinocchio_[index]].shortname();
  else
    return model_.joints[index].shortname();
}

template <typename S>
std::vector<std::string> PinocchioModelTpl<S>::getJointTypes(bool user) const {
  std::vector<std::string> ret;
  const auto njoints = user ? user_joint_names_.size() : model_.joints.size();
  for (size_t i = 0; i < njoints; i++) ret.push_back(getJointType(i, user));
  return ret;
}

template <typename S>
VectorXi PinocchioModelTpl<S>::getJointDims(bool user) const {
  if (user)
    return nvs_;
  else {
    auto ret = VectorXi(model_.nvs.size());
    for (size_t i = 0; i < model_.nvs.size(); i++) ret[i] = model_.nvs[i];
    return ret;
  }
}

template <typename S>
MatrixX<S> PinocchioModelTpl<S>::getJointLimit(size_t index, bool user) const {
  const auto joint_type = getJointType(index, user);
  const size_t pinocchio_idx = user ? joint_index_user2pinocchio_[index] : index;
  const size_t start_idx = model_.idx_qs[pinocchio_idx], nq = model_.nqs[pinocchio_idx],
               dim_joint = getJointDim(index, user);
  ASSERT(dim_joint == 1, "Only support joint with dim 1 but joint" +
                             getJointNames(user)[index] + " has dim " +
                             std::to_string(dim_joint));
  MatrixX<S> ret;
  if (joint_type[pinocchio::joint_type_prefix.size()] == 'P' ||
      (joint_type[pinocchio::joint_type_prefix.size()] == 'R' &&
       joint_type[pinocchio::joint_type_prefix.size() + 1] != 'U')) {
    ret = MatrixX<S>(nq, 2);
    for (size_t j = 0; j < nq; j++) {
      ret(j, 0) = model_.lowerPositionLimit[start_idx + j];
      ret(j, 1) = model_.upperPositionLimit[start_idx + j];
    }
  } else if (joint_type[pinocchio::joint_type_prefix.size()] == 'R' &&
             joint_type[pinocchio::joint_type_prefix.size() + 1] == 'U') {
    ret = MatrixX<S>(1, 2);
    ret(0, 0) = -3.14159265359, ret(0, 1) = 3.14159265359;
  }
  return ret;
}

template <typename S>
std::vector<MatrixX<S>> PinocchioModelTpl<S>::getJointLimits(bool user) const {
  std::vector<MatrixX<S>> ret;
  const auto njoints = user ? user_joint_names_.size() : model_.joints.size();
  for (size_t i = 0; i < njoints; i++) ret.push_back(getJointLimit(i, user));
  return ret;
}

template <typename S>
VectorXi PinocchioModelTpl<S>::getJointParents(bool user) const {
  if (user)
    return parents_;
  else {
    auto ret = VectorXi(model_.parents.size());
    for (size_t i = 0; i < model_.parents.size(); i++) ret[i] = model_.parents[i];
    return ret;
  }
}

template <typename S>
void PinocchioModelTpl<S>::printFrames() const {
  print_info("Joint dim ", model_.joints.size(), " ", model_.nv, " ", model_.nvs.size(),
             " ", model_.idx_vs.size());
  print_info("Joint Tangent dim ", model_.nq, " ", model_.nqs.size(), " ",
             model_.idx_qs.size());
  print_info("Joint Limit ", model_.lowerPositionLimit.transpose(), " ",
             model_.upperPositionLimit.transpose());
  for (size_t i = 0; i < model_.frames.size(); i++) {
    std::string type_name {"NONE"};
    const auto frame = model_.frames[i];
    if (frame.type == pinocchio::OP_FRAME)
      type_name = "OP_FRAME";
    else if (frame.type == pinocchio::JOINT)
      type_name = "JOINT";
    else if (frame.type == pinocchio::FIXED_JOINT)
      type_name = "FIXED_JOINT";
    else if (frame.type == pinocchio::BODY)
      type_name = "BODY";
    else if (frame.type == pinocchio::SENSOR)
      type_name = "BODY";
    print_info("Frame ", i, " ", frame.name, " ", frame.parent, " ", type_name, " ",
               frame.previousFrame);
  }
}

template <typename S>
std::vector<std::string> PinocchioModelTpl<S>::getChainJointName(
    const std::string &end_effector) const {
  const auto frame_id = model_.getBodyId(end_effector);
  const std::vector<std::size_t> index_pinocchio =
      model_.supports[model_.frames[frame_id].parent];
  std::vector<std::string> ret;
  for (const auto &index : index_pinocchio)
    if (joint_index_pinocchio2user_[index] != -1) ret.push_back(model_.names[index]);
  return ret;
}

template <typename S>
std::vector<std::size_t> PinocchioModelTpl<S>::getChainJointIndex(
    const std::string &end_effector) const {
  const auto frame_id = model_.getBodyId(end_effector);
  const std::vector<std::size_t> index_pinocchio =
      model_.supports[model_.frames[frame_id].parent];
  std::vector<std::size_t> ret;
  for (const auto &index : index_pinocchio)
    if (joint_index_pinocchio2user_[index] != -1)
      ret.push_back(joint_index_pinocchio2user_[index]);
  return ret;
}

template <typename S>
VectorX<S> PinocchioModelTpl<S>::qposUser2Pinocchio(const VectorX<S> &q_user) const {
  VectorX<S> q_pinocchio(model_.nq);
  size_t count = 0;
  for (size_t i = 0; i < static_cast<size_t>(joint_index_user2pinocchio_.size()); i++) {
    const auto start_idx = model_.idx_qs[joint_index_user2pinocchio_[i]];
    switch (nqs_[i]) {
      case 0:  // FIX
        break;
      case 1:  // REVOLUTE OR PRISMATIC
        ASSERT(start_idx < model_.nq, "posS2P out of bound");
        q_pinocchio[start_idx] = q_user[count];
        break;
      case 2:  // CONTINUOUS
        ASSERT(start_idx + 1 < model_.nq, "posS2P out of bound");
        q_pinocchio[start_idx] = std::cos(q_user(count));
        q_pinocchio[start_idx + 1] = std::sin(q_user(count));
        break;
      default:
        throw std::runtime_error(
            "Unsupported joint in computation. Currently support: fixed, revolute, "
            "prismatic");
    }
    count += nvs_[i];
  }
  ASSERT(count == static_cast<size_t>(q_user.size()), "Qpos user2pinocchio failed");
  return q_pinocchio;
}

template <typename S>
VectorX<S> PinocchioModelTpl<S>::qposPinocchio2User(
    const VectorX<S> &q_pinocchio) const {
  VectorX<S> q_user(model_.nv);
  size_t count = 0;
  for (size_t i = 0; i < static_cast<size_t>(joint_index_user2pinocchio_.size()); i++) {
    const auto start_idx = model_.idx_qs[joint_index_user2pinocchio_[i]];
    switch (nqs_[i]) {
      case 0:  // FIX
        break;
      case 1:  // REVOLUTE OR PRISMATIC
        ASSERT(count < static_cast<size_t>(model_.nv), "posP2S out of bound");
        q_user[count] = q_pinocchio[start_idx];
        break;
      case 2:  // CONTINUOUS
        ASSERT(count < static_cast<size_t>(model_.nv), "posS2P out of bound");
        q_user[count] = std::atan2(q_pinocchio[start_idx + 1], q_pinocchio[start_idx]);
        break;
      default:
        throw std::runtime_error(
            "Unsupported joint in computation. Currently support: fixed, revolute, "
            "prismatic");
    }
    count += nvs_[i];
  }
  ASSERT(count == static_cast<size_t>(q_user.size()), "Qpos pinocchio2user failed");
  return q_user;
}

template <typename S>
VectorX<S> PinocchioModelTpl<S>::getRandomConfiguration() const {
  return qposPinocchio2User(pinocchio::randomConfiguration(model_));
}

template <typename S>
void PinocchioModelTpl<S>::computeForwardKinematics(const VectorX<S> &qpos) {
  pinocchio::forwardKinematics(model_, data_, qposUser2Pinocchio(qpos));
}

template <typename S>
Isometry3<S> PinocchioModelTpl<S>::getLinkPose(size_t index) const {
  ASSERT(index < static_cast<size_t>(link_index_user2pinocchio_.size()),
         "The link index is out of bound!");
  const auto frame = link_index_user2pinocchio_[index];
  const auto parent_joint = model_.frames[frame].parent;

  const auto link2joint = model_.frames[frame].placement;
  const auto joint2world = data_.oMi[parent_joint];
  const auto link2world = joint2world * link2joint;
  return toIsometry(link2world);
}

template <typename S>
Isometry3<S> PinocchioModelTpl<S>::getJointPose(size_t index) const {
  ASSERT(index < static_cast<size_t>(joint_index_user2pinocchio_.size()),
         "The link index is out of bound!");
  const auto frame = joint_index_user2pinocchio_[index];

  const auto joint2world = data_.oMi[frame];
  return toIsometry(joint2world);
}

template <typename S>
void PinocchioModelTpl<S>::computeFullJacobian(const VectorX<S> &qpos) {
  ASSERT(static_cast<size_t>(qpos.size()) == static_cast<size_t>(model_.nv),
         "qpos size mismatch");
  pinocchio::computeJointJacobians(model_, data_, qposUser2Pinocchio(qpos));
}

template <typename S>
Matrix6X<S> PinocchioModelTpl<S>::getLinkJacobian(size_t index, bool local) const {
  ASSERT(index < static_cast<size_t>(link_index_user2pinocchio_.size()),
         "link index out of bound");
  const auto frameId = link_index_user2pinocchio_[index];
  const auto jointId = model_.frames[frameId].parent;

  const auto link2joint = model_.frames[frameId].placement;
  const auto joint2world = data_.oMi[jointId];
  const auto link2world = joint2world * link2joint;

  Matrix6X<S> J(6, model_.nv);
  J.fill(0);
  pinocchio::getJointJacobian(model_, data_, jointId, pinocchio::ReferenceFrame::WORLD,
                              J);
  if (local) J = link2world.toActionMatrixInverse() * J;
  return J * v_map_user2pinocchio_;
}

template <typename S>
Matrix6X<S> PinocchioModelTpl<S>::computeSingleLinkJacobian(const VectorX<S> &qpos,
                                                            size_t index, bool local) {
  ASSERT(index < static_cast<size_t>(link_index_user2pinocchio_.size()),
         "link index out of bound");
  ASSERT(static_cast<size_t>(qpos.size()) == static_cast<size_t>(model_.nv),
         "qpos size mismatch");
  const auto frameId = link_index_user2pinocchio_[index];
  Matrix6X<S> J(6, model_.nv);
  J.fill(0);
  auto rf = local ? pinocchio::LOCAL : pinocchio::WORLD;
  pinocchio::computeFrameJacobian(model_, data_, qposUser2Pinocchio(qpos), frameId, rf,
                                  J);
  return J * v_map_user2pinocchio_;
}

template <typename S>
std::tuple<VectorX<S>, bool, Vector6<S>> PinocchioModelTpl<S>::computeIKCLIK(
    size_t index, const Pose<S> &pose, const VectorX<S> &q_init,
    const std::vector<bool> &mask, double eps, int max_iter, double dt, double damp) {
  ASSERT(index < static_cast<size_t>(link_index_user2pinocchio_.size()),
         "link index out of bound");
  const auto frameId = link_index_user2pinocchio_[index];
  const auto jointId = model_.frames[frameId].parent;
  const auto link2joint = model_.frames[frameId].placement;

  const pinocchio::SE3Tpl<S> link_pose {pose.q, pose.p};
  const pinocchio::SE3Tpl<S> joint_pose = link_pose * link2joint.inverse();
  VectorX<S> q = qposUser2Pinocchio(q_init);  // pinocchio::neutral(model);
  Matrix6X<S> J(6, model_.nv);
  J.setZero();

  bool success = false;
  Vector6<S> err;
  VectorX<S> v(model_.nv);

  int mask_size = mask.size();
  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model_, data_, q);
    const pinocchio::SE3Tpl<S> dMi = joint_pose.actInv(data_.oMi[jointId]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= max_iter) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model_, data_, q, jointId, J);

    // mask out certain joints
    if (mask_size != 0)
      for (int j = 0; j < mask_size; j++)
        if (mask[j]) {
          const int u = joint_index_user2pinocchio_[j] - 1;
          for (int k = 0; k < 6; k++) J(k, u) = 0;
        }
    Matrix6<S> JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model_, q, v * dt);
  }
  return {qposPinocchio2User(q), success, err};
}

template <typename S>
std::tuple<VectorX<S>, bool, Vector6<S>> PinocchioModelTpl<S>::computeIKCLIKJL(
    size_t index, const Pose<S> &pose, const VectorX<S> &q_init,
    const VectorX<S> &q_min, const VectorX<S> &q_max, double eps, int max_iter,
    double dt, double damp) {
  ASSERT(index < static_cast<size_t>(link_index_user2pinocchio_.size()),
         "link index out of bound");
  const auto frameId = link_index_user2pinocchio_[index];
  const auto jointId = model_.frames[frameId].parent;
  const auto link2joint = model_.frames[frameId].placement;

  const pinocchio::SE3Tpl<S> link_pose {pose.q, pose.p};
  const pinocchio::SE3Tpl<S> joint_pose = link_pose * link2joint.inverse();
  VectorX<S> q = qposUser2Pinocchio(q_init);  // pinocchio::neutral(model);
  const VectorX<S> qmin = qposUser2Pinocchio(q_min);
  const VectorX<S> qmax = qposUser2Pinocchio(q_max);
  Matrix6X<S> J(6, model_.nv);
  J.setZero();

  bool success = false;
  Vector6<S> err;
  VectorX<S> v(model_.nv);

  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model_, data_, q);
    const pinocchio::SE3Tpl<S> dMi = joint_pose.actInv(data_.oMi[jointId]);
    err = pinocchio::log6(dMi).toVector();
    if (err.norm() < eps) {
      success = true;
      break;
    }
    if (i >= max_iter) {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model_, data_, q, jointId, J);
    Matrix6<S> JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model_, q, v * dt);
    for (int j = 0; j < q.rows(); j++) {
      if (q[j] < qmin[j]) {
        if (abs(qmin[j] + 3.1415926) < 1e-3)
          q[j] = 3.1415926;
        else
          q[j] = qmin[j];
      }
      if (q[j] > qmax[j]) {
        if (abs(qmin[j] - 3.1415926) < 1e-3)
          q[j] = -3.1415926;
        else
          q[j] = qmax[j];
      }
    }
  }
  return {qposPinocchio2User(q), success, err};
}

}  // namespace mplib::kinematics::pinocchio
