#pragma once

#include <functional>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>
#include <pinocchio/fwd.hpp>

#include "color_printing.h"
#include "planning_world.h"

namespace ob = ompl::base;
// namespace oc = ompl::control;
namespace og = ompl::geometric;

template <typename S>
std::vector<S> compoundstate2vector(const ob::State *state_raw,
                                    ob::SpaceInformation *const &si_) {
  auto state = state_raw->as<ob::CompoundState>();
  std::vector<S> ret;
  auto si = si_->getStateSpace()->as<ob::CompoundStateSpace>();

  for (size_t i = 0; i < si->getSubspaceCount(); i++) {
    auto subspace(si->getSubspace(i));
    size_t n;
    switch (subspace->getType()) {
      case ob::STATE_SPACE_REAL_VECTOR:
        n = subspace->as<ob::RealVectorStateSpace>()->getDimension();
        for (size_t j = 0; j < n; j++)
          ret.push_back(
              (S)(*state)[i]->as<ob::RealVectorStateSpace::StateType>()->values[j]);
        break;
      case ob::STATE_SPACE_SO2:
        ret.push_back((S)(*state)[i]->as<ob::SO2StateSpace::StateType>()->value);
        break;
      default:
        throw std::invalid_argument("Unhandled subspace type.");
        break;
    }
  }
  return ret;
}

template <typename S>
std::vector<S> rvssstate2vector(const ob::State *state_raw,
                                ob::SpaceInformation *const &si_) {
  auto dim = si_->getStateDimension();
  auto state = state_raw->as<ob::ProjectedStateSpace::StateType>();
  std::vector<S> ret;
  for (size_t i = 0; i < dim; i++) {
    ret.push_back((S)(*state)[i]);
  }
  return ret;
}

template <typename IN_TYPE, typename OUT_TYPE>
std::vector<OUT_TYPE> eigen2vector(const Eigen::Matrix<IN_TYPE, Eigen::Dynamic, 1> &x) {
  std::vector<OUT_TYPE> ret;
  for (auto i = 0; i < x.rows(); i++) ret.push_back((OUT_TYPE)x[i]);
  return ret;
}

template <typename IN_TYPE, typename OUT_TYPE>
Eigen::Matrix<OUT_TYPE, Eigen::Dynamic, 1> vector2eigen(const std::vector<IN_TYPE> &x) {
  Eigen::Matrix<OUT_TYPE, Eigen::Dynamic, 1> ret(x.size());
  for (size_t i = 0; i < x.size(); i++) ret[i] = (OUT_TYPE)x[i];
  return ret;
}

template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, 1> state2eigen(const ob::State *state_raw,
                                                ob::SpaceInformation *const &si_,
                                                bool is_rvss = false) {
  std::vector<S> vec_ret = is_rvss ? rvssstate2vector<S>(state_raw, si_)
                                   : compoundstate2vector<S>(state_raw, si_);
  auto ret = vector2eigen<S, S>(vec_ret);
  return ret;
}

struct FixedJoint {
  size_t articulation_idx;  // which robot in the planning world does the fixed joint
                            // belong to?
  size_t joint_idx;         // what is the index of the joint you want it fixed?
  double value;  // what is the value of the fixed joint?  we are actively trying to get
                 // rid of the S template

  FixedJoint(size_t articulation_idx, size_t joint_idx, double value)
      : articulation_idx(articulation_idx), joint_idx(joint_idx), value(value) {}

  bool operator==(const FixedJoint &other) const {
    return articulation_idx == other.articulation_idx && joint_idx == other.joint_idx;
  }

  bool operator<(const FixedJoint &other) const {
    return articulation_idx < other.articulation_idx ||
           (articulation_idx == other.articulation_idx && joint_idx < other.joint_idx);
  }
};

using FixedJoints = std::set<FixedJoint>;

bool is_fixed_joint(const FixedJoints &fixed_joints, size_t articulation_idx,
                    size_t joint_idx);

Eigen::VectorXd remove_fixed_joints(const FixedJoints &fixed_joints,
                                    const Eigen::VectorXd &state);

Eigen::VectorXd add_fixed_joints(const FixedJoints &fixed_joints,
                                 const Eigen::VectorXd &state);

template <typename S>
class ValidityCheckerTpl : public ob::StateValidityChecker {
  using VectorX = Eigen::Matrix<S, Eigen::Dynamic, 1>;
  PlanningWorldTplPtr<S> world_;
  bool is_rvss_;
  FixedJoints fixed_joints_;

 public:
  ValidityCheckerTpl(PlanningWorldTplPtr<S> world, const ob::SpaceInformationPtr &si,
                     bool is_rvss, const FixedJoints &fixed_joints = FixedJoints())
      : ob::StateValidityChecker(si),
        world_(world),
        is_rvss_(is_rvss),
        fixed_joints_(fixed_joints) {}

  void update_fixed_joints(const FixedJoints &fixed_joints) {
    this->fixed_joints_ = fixed_joints;
  }

  bool _isValid(VectorX state) const {
    world_->setQposAll(add_fixed_joints(fixed_joints_, state));
    return !world_->collide();
  }

  bool isValid(const ob::State *state_raw) const {
    auto state = state2eigen<S>(state_raw, si_, is_rvss_);
    return _isValid(state);
  }
};

class GeneralConstraint : public ob::Constraint {
  std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> f_, j_;

 public:
  GeneralConstraint(
      size_t dim,
      const std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)>
          &f,
      const std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)>
          &j)
      : ob::Constraint(dim, 1), f_(f), j_(j) {}

  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                Eigen::Ref<Eigen::VectorXd> out) const override {
    f_(x, out);
  }

  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                Eigen::Ref<Eigen::MatrixXd> out) const override {
    j_(x, out);
  }
};

template <typename S>
using ValidityCheckerTplPtr = std::shared_ptr<ValidityCheckerTpl<S>>;

using ValidityCheckerdPtr = ValidityCheckerTplPtr<double>;
using ValidityCheckerfPtr = ValidityCheckerTplPtr<float>;
using ValidityCheckerd = ValidityCheckerTpl<double>;
using ValidityCheckerf = ValidityCheckerTpl<float>;

/// OMPL Planner
template <typename S>
class OMPLPlannerTpl {
  using CompoundStateSpacePtr = std::shared_ptr<ob::CompoundStateSpace>;
  using SpaceInformationPtr = std::shared_ptr<ob::SpaceInformation>;
  using ProblemDefinitionPtr = std::shared_ptr<ob::ProblemDefinition>;

  using CompoundStateSpace = ob::CompoundStateSpace;
  using SpaceInformation = ob::SpaceInformation;
  using ProblemDefinition = ob::ProblemDefinition;
  using ValidityChecker = ValidityCheckerTpl<S>;
  using ValidityCheckerPtr = ValidityCheckerTplPtr<S>;

  DEFINE_TEMPLATE_EIGEN(S)

  std::shared_ptr<ob::RealVectorStateSpace> p_ambient_space_;
  std::shared_ptr<ob::ProjectedStateSpace> p_constrained_space_;
  CompoundStateSpacePtr cs_;
  ob::StateSpacePtr state_space_;
  std::shared_ptr<ompl::geometric::SimpleSetup> ss_;
  SpaceInformationPtr p_compound_si_;
  SpaceInformationPtr p_constrained_si_;
  SpaceInformationPtr si_;
  PlanningWorldTplPtr<S> world_;
  ValidityCheckerTplPtr<S> valid_checker_;
  size_t dim_;
  std::vector<S> lower_joint_limits_, upper_joint_limits_;
  std::vector<bool> is_revolute_;
  // if not empty, then we need to update the state space
  FixedJoints last_fixed_joints_;

  /** Certain joint configurations are the same if some joints are the same fmod 2pi. */
  std::shared_ptr<ob::GoalStates> make_goal_states(
      const std::vector<VectorX> &goal_states);

  /** build a real vector state space for the robot */
  void build_constrained_ambient_state_space();

  /**
   * ss depends on si so every time we change that need to redo it.
   * also need to store the space type since ompl loses it -_-
   */
  void update_ss(bool is_rvss = false);

  void _simplify_path(og::PathGeometric &path);  // keep this private to avoid confusion

 public:
  /**
   * Construct an OMPLPlanner from a PlanningWorld
   *
   * @param world: planning world
   */
  OMPLPlannerTpl(const PlanningWorldTplPtr<S> &world, int robot_idx = 0);

  VectorX random_sample_nearby(const VectorX &start_state);

  /**
   * @brief Build a new state space given the current planning world
   *        and a set of fixed joints
   *
   * @param fixed_joints: a vector of FixedJoint
   */
  void build_compound_state_space(const FixedJoints &fixed_joints = FixedJoints());

  PlanningWorldTplPtr<S> get_world() { return world_; }

  size_t get_dim() { return dim_; }

  /**
   * Simplify the provided path.
   *
   * @param path: path to be simplified (numpy array of shape (n, dim))
   * @return: simplified path
   */
  Eigen::MatrixXd simplify_path(Eigen::MatrixXd &path);

  /**
   * Plan a path from start state to goal states.
   *
   * @param start_state: start state of the movegroup joints
   * @param goal_states: list of goal states. Planner will stop when one of them is
   *                     reached
   * @param planner_name: name of the planner pick between {RRTConnect, RRT*}
   * @param time: planning time limit
   * @param range: planning range (for RRT family of planners and represents the maximum
   *               step size)
   * @param verbose: print debug information
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
   * @return: pair of planner status and path. If planner succeeds, status is "Exact
   *          solution."
   */
  std::pair<std::string, Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>> plan(
      const VectorX &start_state, const std::vector<VectorX> &goal_states,
      const std::string &planner_name = "RRTConnect", const double &time = 1.0,
      const double &range = 0.0, const bool verbose = false,
      const FixedJoints &fixed_joints = FixedJoints(),
      const bool no_simplification = false,
      const std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)>
          &constraint_function = nullptr,
      const std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)>
          &constraint_jacobian = nullptr,
      double constraint_tolerance = 1e-3);
};

template <typename S>
using OMPLPlannerTplPtr = std::shared_ptr<ValidityCheckerTpl<S>>;

using OMPLPlannerTpldPtr = OMPLPlannerTplPtr<double>;
using OMPLPlannerTplfPtr = OMPLPlannerTplPtr<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplf = OMPLPlannerTpl<float>;
