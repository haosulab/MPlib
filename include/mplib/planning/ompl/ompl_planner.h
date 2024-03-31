#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include "mplib/macros/class_forward.h"
#include "mplib/planning/ompl/fixed_joint.h"
#include "mplib/planning/ompl/types.h"
#include "mplib/planning/ompl/validity_checker.h"
#include "mplib/planning_world.h"
#include "mplib/types.h"

namespace mplib::planning::ompl {

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
   *   reached
   * @param time: planning time limit
   * @param range: planning range (for RRT family of planners and represents the maximum
   *   step size)
   * @param fixed_joints: list of fixed joints not considered in planning for this
   *   particular call
   * @param simplify: whether the path will be simplified by calling ``_simplifyPath()``
   *   (constained planning does not support simplification)
   * @param constraint_function: a R^d to R^1 function that evals to 0 when constraint
   *   is satisfied. Constraint ignored if fixed joints not empty
   * @param constraint_jacobian: the jacobian of the constraint w.r.t. the joint angles
   * @param constraint_tolerance: tolerance of what level of deviation from 0 is
   *   acceptable
   * @param verbose: print debug information. Default: ``false``.
   * @return: pair of planner status and path. If planner succeeds, status is "Exact
   *   solution."
   */
  std::pair<std::string, MatrixX<S>> plan(
      const VectorX<S> &start_state, const std::vector<VectorX<S>> &goal_states,
      double time = 1.0, double range = 0.0,
      const FixedJointsTpl<S> &fixed_joints = FixedJointsTpl<S>(), bool simplify = true,
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
   * Build a new state space given the current planning world and a set of fixed joints
   *
   * @param fixed_joints: a vector of FixedJoint
   */
  void buildCompoundStateSpace(
      const FixedJointsTpl<S> &fixed_joints = FixedJointsTpl<S>());

  VectorX<S> randomSampleNearby(const VectorX<S> &start_state) const;

  /** Certain joint configurations are the same if some joints are the same fmod 2pi. */
  ob::GoalStatesPtr makeGoalStates(const std::vector<VectorX<S>> &goal_states) const;

  void _simplifyPath(og::PathGeometric &path) const;

  PlanningWorldTplPtr<S> world_;

  ob::RealVectorStateSpacePtr ambient_space_;

  ob::CompoundStateSpacePtr cs_;
  ob::StateSpacePtr state_space_;
  std::vector<S> lower_joint_limits_, upper_joint_limits_;
  std::vector<bool> is_revolute_;
  size_t dim_ {};
  ob::SpaceInformationPtr si_;
  ob::SpaceInformationPtr compound_si_;
  ValidityCheckerTplPtr<S> valid_checker_;
  og::SimpleSetupPtr ss_;

  ob::ProjectedStateSpacePtr constrained_space_;
  ob::SpaceInformationPtr constrained_si_;

  // if not empty, then we need to update the state space
  FixedJointsTpl<S> last_fixed_joints_;
};

// Common Type Alias ===================================================================
using OMPLPlannerTplf = OMPLPlannerTpl<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplfPtr = OMPLPlannerTplPtr<float>;
using OMPLPlannerTpldPtr = OMPLPlannerTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_OMPL_PLANNER(S) extern template class OMPLPlannerTpl<S>

DECLARE_TEMPLATE_OMPL_PLANNER(float);
DECLARE_TEMPLATE_OMPL_PLANNER(double);

}  // namespace mplib::planning::ompl
