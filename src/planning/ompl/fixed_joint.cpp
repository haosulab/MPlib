#include "mplib/planning/ompl/fixed_joint.h"

namespace mplib::planning::ompl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_FIXED_JOINT(S)                                            \
  template bool isFixedJoint<S>(const FixedJointsTpl<S> &fixed_joints,            \
                                size_t articulation_idx, size_t joint_idx);       \
  template VectorX<S> addFixedJoints<S>(const FixedJointsTpl<S> &fixed_joints,    \
                                        const VectorX<S> &state);                 \
  template VectorX<S> removeFixedJoints<S>(const FixedJointsTpl<S> &fixed_joints, \
                                           const VectorX<S> &state)

DEFINE_TEMPLATE_FIXED_JOINT(float);
DEFINE_TEMPLATE_FIXED_JOINT(double);

template <typename S>
bool isFixedJoint(const FixedJointsTpl<S> &fixed_joints, size_t articulation_idx,
                  size_t joint_idx) {
  for (const auto &fixed_joint : fixed_joints)
    if (fixed_joint.articulation_idx == articulation_idx &&
        fixed_joint.joint_idx == joint_idx)
      return true;
  return false;
}

template <typename S>
VectorX<S> addFixedJoints(const FixedJointsTpl<S> &fixed_joints,
                          const VectorX<S> &state) {
  auto fixed_itr = fixed_joints.begin();
  VectorX<S> ret(fixed_joints.size() + state.rows());
  size_t j = 0;
  for (auto i = 0; i < ret.rows(); i++) {
    if (isFixedJoint<S>(fixed_joints, 0, i)) {
      ret[i] = fixed_itr->value;
      ++fixed_itr;
    } else {
      ret[i] = state[j++];
    }
  }
  return ret;
}

template <typename S>
VectorX<S> removeFixedJoints(const FixedJointsTpl<S> &fixed_joints,
                             const VectorX<S> &state) {
  VectorX<S> ret(state.rows() - fixed_joints.size());
  size_t cnt = 0;
  for (auto i = 0; i < state.rows(); i++) {
    if (isFixedJoint<S>(fixed_joints, 0, i))
      continue;  // TODO[xinsong] only support one robot rn
    ret[cnt++] = state[i];
  }
  return ret;
}

}  // namespace mplib::planning::ompl
