#pragma once

#include <set>

#include "mplib/types.h"

namespace mplib::planning::ompl {

template <typename S>
struct FixedJointTpl {
  FixedJointTpl(size_t articulation_idx, size_t joint_idx, S value)
      : articulation_idx(articulation_idx), joint_idx(joint_idx), value(value) {}

  bool operator==(const FixedJointTpl &other) const {
    return articulation_idx == other.articulation_idx && joint_idx == other.joint_idx;
  }

  bool operator<(const FixedJointTpl &other) const {
    return articulation_idx < other.articulation_idx ||
           (articulation_idx == other.articulation_idx && joint_idx < other.joint_idx);
  }

  /// index of the articulated model in the PlanningWorld that the fixed joint belong to
  size_t articulation_idx {};
  size_t joint_idx {};  ///< the index of the fixed joint
  S value {};           ///< the value of the fixed joint
};

template <typename S>
using FixedJointsTpl = std::set<FixedJointTpl<S>>;

template <typename S>
bool isFixedJoint(const FixedJointsTpl<S> &fixed_joints, size_t articulation_idx,
                  size_t joint_idx);

template <typename S>
VectorX<S> addFixedJoints(const FixedJointsTpl<S> &fixed_joints,
                          const VectorX<S> &state);

template <typename S>
VectorX<S> removeFixedJoints(const FixedJointsTpl<S> &fixed_joints,
                             const VectorX<S> &state);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_FIXED_JOINT(S)                                               \
  extern template bool isFixedJoint<S>(const FixedJointsTpl<S> &fixed_joints,         \
                                       size_t articulation_idx, size_t joint_idx);    \
  extern template VectorX<S> addFixedJoints<S>(const FixedJointsTpl<S> &fixed_joints, \
                                               const VectorX<S> &state);              \
  extern template VectorX<S> removeFixedJoints<S>(                                    \
      const FixedJointsTpl<S> &fixed_joints, const VectorX<S> &state)

DECLARE_TEMPLATE_FIXED_JOINT(float);
DECLARE_TEMPLATE_FIXED_JOINT(double);

}  // namespace mplib::planning::ompl
