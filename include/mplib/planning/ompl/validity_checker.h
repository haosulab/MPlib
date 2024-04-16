#pragma once

#include "mplib/macros/class_forward.h"
#include "mplib/planning/ompl/fixed_joint.h"
#include "mplib/planning/ompl/ompl_utils.h"
#include "mplib/planning/ompl/types.h"
#include "mplib/planning_world.h"
#include "mplib/types.h"

namespace mplib::planning::ompl {

// ValidityCheckerTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(ValidityCheckerTpl);

template <typename S>
class ValidityCheckerTpl : public ob::StateValidityChecker {
 public:
  ValidityCheckerTpl(PlanningWorldTplPtr<S> world, const ob::SpaceInformationPtr &si,
                     bool is_rvss,
                     const FixedJointsTpl<S> &fixed_joints = FixedJointsTpl<S>())
      : ob::StateValidityChecker(si),
        world_(world),
        is_rvss_(is_rvss),
        fixed_joints_(fixed_joints) {}

  bool _isValid(const VectorX<S> &state) const {
    world_->setQposAll(addFixedJoints(fixed_joints_, state));
    return !world_->isStateColliding();
  }

  bool isValid(const ob::State *state_raw) const {
    return _isValid(state2Eigen<S>(state_raw, si_, is_rvss_));
  }

 private:
  PlanningWorldTplPtr<S> world_;
  bool is_rvss_ {};
  FixedJointsTpl<S> fixed_joints_;
};

// Common Type Alias ===================================================================
using ValidityCheckerf = ValidityCheckerTpl<float>;
using ValidityCheckerd = ValidityCheckerTpl<double>;
using ValidityCheckerfPtr = ValidityCheckerTplPtr<float>;
using ValidityCheckerdPtr = ValidityCheckerTplPtr<double>;

}  // namespace mplib::planning::ompl
