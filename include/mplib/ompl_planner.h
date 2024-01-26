#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "macros_utils.h"
#include "planning_world.h"
#include "types.h"

namespace mplib::ompl {

template <typename S>
std::vector<S> compoundState2Vector(const State *state_raw, const SpaceInformation *si);

template <typename S>
std::vector<S> rvssState2Vector(const State *state_raw, const SpaceInformation *si);

template <typename IN_TYPE, typename OUT_TYPE>
std::vector<OUT_TYPE> eigen2Vector(const VectorX<IN_TYPE> &v) {
  std::vector<OUT_TYPE> ret;
  for (const auto &x : v) ret.push_back(static_cast<OUT_TYPE>(x));
  return ret;
}

template <typename IN_TYPE, typename OUT_TYPE>
VectorX<OUT_TYPE> vector2Eigen(const std::vector<IN_TYPE> &v) {
  VectorX<OUT_TYPE> ret(v.size());
  for (size_t i = 0; i < v.size(); i++) ret[i] = static_cast<OUT_TYPE>(v[i]);
  return ret;
}

/**
 * Convert a ompl::base::State to Eigen::VectorX.
 *
 * @param state_raw: pointer to a raw state.
 * @param si: pointer to ompl::base::SpaceInformation.
 * @param is_rvss: whether the state space is an ompl::base::RealVectorStateSpace.
 *    If ``true``, we are using constrained planning.
 * @return: an Eigen::VectorX of the ompl::base::State.
 */
template <typename S>
VectorX<S> state2Eigen(const State *state_raw, const SpaceInformation *si,
                       bool is_rvss = false);

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

  size_t articulation_idx;  // which robot in the planning world does the fixed joint
                            // belong to?
  size_t joint_idx;         // what is the index of the joint you want it fixed?
  S value;                  // what is the value of the fixed joint?
};

template <typename S>
bool isFixedJoint(const FixedJointsTpl<S> &fixed_joints, size_t articulation_idx,
                  size_t joint_idx);

template <typename S>
VectorX<S> removeFixedJoints(const FixedJointsTpl<S> &fixed_joints,
                             const VectorX<S> &state);

template <typename S>
VectorX<S> addFixedJoints(const FixedJointsTpl<S> &fixed_joints,
                          const VectorX<S> &state);

// ValidityCheckerTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(ValidityCheckerTpl);

template <typename S>
class ValidityCheckerTpl : public StateValidityChecker {
 public:
  ValidityCheckerTpl(PlanningWorldTplPtr<S> world, const SpaceInformationPtr &si,
                     bool is_rvss,
                     const FixedJointsTpl<S> &fixed_joints = FixedJointsTpl<S>())
      : StateValidityChecker(si),
        world_(world),
        is_rvss_(is_rvss),
        fixed_joints_(fixed_joints) {}

  bool _isValid(const VectorX<S> &state) const {
    world_->setQposAll(addFixedJoints(fixed_joints_, state));
    return !world_->collide();
  }

  bool isValid(const State *state_raw) const {
    return _isValid(state2Eigen<S>(state_raw, si_, is_rvss_));
  }

 private:
  PlanningWorldTplPtr<S> world_;
  bool is_rvss_;
  FixedJointsTpl<S> fixed_joints_;
};

// Common Type Alias ===================================================================
using ValidityCheckerf = ValidityCheckerTpl<float>;
using ValidityCheckerd = ValidityCheckerTpl<double>;
using ValidityCheckerfPtr = ValidityCheckerTplPtr<float>;
using ValidityCheckerdPtr = ValidityCheckerTplPtr<double>;

class GeneralConstraint : public Constraint {
 public:
  GeneralConstraint(
      size_t dim, const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)> &f,
      const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)> &j)
      : Constraint(dim, 1), f_(f), j_(j) {}

  void function(const Eigen::Ref<const VectorXd> &x,
                Eigen::Ref<VectorXd> out) const override {
    f_(x, out);
  }

  void jacobian(const Eigen::Ref<const VectorXd> &x,
                Eigen::Ref<MatrixXd> out) const override {
    j_(x, out);
  }

 private:
  std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)> f_, j_;
};

// OMPLPlannerTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(OMPLPlannerTpl);

/// OMPL Planner
template <typename S>
class OMPLPlannerTpl {
 public:
  /**
   * Construct an OMPLPlanner from a PlanningWorld
   *
   * @param world: planning world
   */
  OMPLPlannerTpl(const PlanningWorldTplPtr<S> &world);

  /**
   * Plan a path from start state to goal states.
   *
   * @param start_state: start state of the movegroup joints
   * @param goal_states: list of goal states. Planner will stop when one of them is
   *                     reached
   * @param planner_name: name of the planner pick between {RRTConnect, RRTstar}
   * @param time: planning time limit
   * @param range: planning range (for RRT family of planners and represents the maximum
   *               step size)
   * @param fixed_joints: list of fixed joints not considered in planning for this
   *                      particular call
   * @param no_simplification: if ``true``, the path will not be simplified (constained
   *                           planning does not support simplification)
   * @param constraint_function: a R^d to R^1 function that evals to 0 when constraint
   *                             is satisfied. Constraint ignored if fixed joints not
   *                             empty
   * @param constraint_jacobian: the jacobian of the constraint w.r.t. the joint angles
   * @param constraint_tolerance: tolerance of what level of deviation from 0 is
   *                              acceptable
   * @param verbose: print debug information. Default: ``false``.
   * @return: pair of planner status and path. If planner succeeds, status is "Exact
   *          solution."
   */
  std::pair<std::string, MatrixX<S>> plan(
      const VectorX<S> &start_state, const std::vector<VectorX<S>> &goal_states,
      const std::string &planner_name = "RRTConnect", double time = 1.0,
      double range = 0.0, const FixedJointsTpl<S> &fixed_joints = FixedJointsTpl<S>(),
      bool no_simplification = false,
      const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)>
          &constraint_function = nullptr,
      const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)>
          &constraint_jacobian = nullptr,
      double constraint_tolerance = 1e-3, bool verbose = false);

  /**
   * Simplify the provided path.
   *
   * @param path: path to be simplified (numpy array of shape (n, dim))
   * @return: simplified path
   */
  MatrixX<S> simplifyPath(const MatrixX<S> &path) const;

 private:
  /** build a real vector state space for the robot */
  void buildConstrainedAmbientStateSpace();

  /**
   * @brief Build a new state space given the current planning world
   *        and a set of fixed joints
   *
   * @param fixed_joints: a vector of FixedJoint
   */
  void buildCompoundStateSpace(
      const FixedJointsTpl<S> &fixed_joints = FixedJointsTpl<S>());

  VectorX<S> randomSampleNearby(const VectorX<S> &start_state) const;

  /** Certain joint configurations are the same if some joints are the same fmod 2pi. */
  GoalStatesPtr makeGoalStates(const std::vector<VectorX<S>> &goal_states) const;

  void _simplifyPath(PathGeometric &path) const;

  PlanningWorldTplPtr<S> world_;

  RealVectorStateSpacePtr ambient_space_;

  CompoundStateSpacePtr cs_;
  StateSpacePtr state_space_;
  std::vector<S> lower_joint_limits_, upper_joint_limits_;
  std::vector<bool> is_revolute_;
  size_t dim_;
  SpaceInformationPtr si_;
  SpaceInformationPtr compound_si_;
  ValidityCheckerTplPtr<S> valid_checker_;
  SimpleSetupPtr ss_;

  ProjectedStateSpacePtr constrained_space_;
  SpaceInformationPtr constrained_si_;

  // if not empty, then we need to update the state space
  FixedJointsTpl<S> last_fixed_joints_;
};

// Common Type Alias ===================================================================
using OMPLPlannerTplf = OMPLPlannerTpl<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplfPtr = OMPLPlannerTplPtr<float>;
using OMPLPlannerTpldPtr = OMPLPlannerTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_OMPL_PLANNER(S)                                               \
  extern template std::vector<S> compoundState2Vector<S>(const State *state_raw,       \
                                                         const SpaceInformation *si);  \
  extern template std::vector<S> rvssState2Vector<S>(const State *state_raw,           \
                                                     const SpaceInformation *si);      \
  extern template VectorX<S> state2Eigen<S>(const State *state_raw,                    \
                                            const SpaceInformation *si, bool is_rvss); \
  extern template bool isFixedJoint<S>(const FixedJointsTpl<S> &fixed_joints,          \
                                       size_t articulation_idx, size_t joint_idx);     \
  extern template VectorX<S> removeFixedJoints<S>(                                     \
      const FixedJointsTpl<S> &fixed_joints, const VectorX<S> &state);                 \
  extern template VectorX<S> addFixedJoints<S>(const FixedJointsTpl<S> &fixed_joints,  \
                                               const VectorX<S> &state);               \
  extern template struct FixedJointTpl<S>;                                             \
  extern template class ValidityCheckerTpl<S>;                                         \
  extern template class OMPLPlannerTpl<S>

DECLARE_TEMPLATE_OMPL_PLANNER(float);
DECLARE_TEMPLATE_OMPL_PLANNER(double);

}  // namespace mplib::ompl
