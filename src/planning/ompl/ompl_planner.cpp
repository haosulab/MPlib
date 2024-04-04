#include "mplib/planning/ompl/ompl_planner.h"

#include <memory>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "mplib/macros/assert.h"
#include "mplib/planning/ompl/general_constraint.h"
#include "mplib/planning/ompl/ompl_utils.h"
#include "mplib/utils/color_printing.h"

namespace mplib::planning::ompl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_OMPL_PLANNER(S) template class OMPLPlannerTpl<S>

DEFINE_TEMPLATE_OMPL_PLANNER(float);
DEFINE_TEMPLATE_OMPL_PLANNER(double);

#define PI 3.14159265359

template <typename S>
OMPLPlannerTpl<S>::OMPLPlannerTpl(const PlanningWorldTplPtr<S> &world) : world_(world) {
  // default is constrained space, builds the ambient space for later
  buildConstrainedAmbientStateSpace();
  buildCompoundStateSpace();
}

template <typename S>
void OMPLPlannerTpl<S>::buildConstrainedAmbientStateSpace() {
  lower_joint_limits_.clear();
  upper_joint_limits_.clear();
  is_revolute_.clear();

  ASSERT(world_->getPlannedArticulations().size() == 1,
         "only support one robot for constrained planning");
  const auto robot = world_->getPlannedArticulations()[0];

  // we assume each joint has only one DoF
  dim_ = robot->getQposDim();  // this getQposDim() is really getMoveGroupQposDim()
  ambient_space_ = std::make_shared<ob::RealVectorStateSpace>(dim_);
  const auto pinocchio_model = robot->getPinocchioModel();
  const auto joint_types = pinocchio_model->getJointTypes();
  const auto indices = robot->getMoveGroupJointIndices();
  ASSERT(dim_ == indices.size(), "movegroupQposDim != size of the movegroup joints");

  ob::RealVectorBounds ambient_space_bounds(dim_);
  for (size_t i = 0; i < dim_; i++) {
    const auto pinocchio_id = indices[i];  // joint_id to pinocchio_id
    const auto joint_type = joint_types[pinocchio_id];
    // P stands for PRISMATIC, R stands for REVOLUTE, U stands for UNBOUNDED (continuous
    // joint)
    if (joint_type[pinocchio::joint_type_prefix.size()] == 'R')
      ASSERT(joint_type[pinocchio::joint_type_prefix.size() + 1] != 'U',
             "Do not support continuous revolute joint for constrained planning");
    if (joint_type[pinocchio::joint_type_prefix.size()] == 'P' ||
        joint_type[pinocchio::joint_type_prefix.size()] == 'R') {
      const auto bound = pinocchio_model->getJointLimit(pinocchio_id);
      ASSERT(bound.rows() == 1,
             "Only support simple joint with dim of joint being one");
      ambient_space_bounds.setLow(i, bound(0, 0));
      ambient_space_bounds.setHigh(i, bound(0, 1));
    }
  }
  ambient_space_->setBounds(ambient_space_bounds);
}

template <typename S>
void OMPLPlannerTpl<S>::buildCompoundStateSpace(const FixedJointsTpl<S> &fixed_joints) {
  cs_ = std::make_shared<ob::CompoundStateSpace>();
  dim_ = 0;
  lower_joint_limits_.clear();
  upper_joint_limits_.clear();
  is_revolute_.clear();

  const auto robots = world_->getPlannedArticulations();
  for (size_t robot_idx = 0; robot_idx < robots.size(); ++robot_idx) {
    const auto robot = robots[robot_idx];
    const size_t dof = robot->getQposDim();  // TODO: only construct move group joints
    const auto pinocchio_model = robot->getPinocchioModel();
    const auto joint_types = pinocchio_model->getJointTypes();
    const auto indices = robot->getMoveGroupJointIndices();
    ASSERT(dof == indices.size(), "QposDim != size of the movegroup joints");

    size_t dim_cnt = 0;
    size_t ignored_dim_cnt = 0;
    for (size_t i = 0; i < dof; i++) {
      // if we have already fixed this joint, we don't need to add it to the state space
      if (isFixedJoint<S>(fixed_joints, robot_idx, i)) {
        ++ignored_dim_cnt;
        continue;
      }

      const auto pinocchio_id = indices[i];
      const auto joint_type = joint_types[pinocchio_id];
      // PRISMATIC or REVOLUTE joint
      if (joint_type[pinocchio::joint_type_prefix.size()] == 'P' ||
          (joint_type[pinocchio::joint_type_prefix.size()] == 'R' &&
           joint_type[pinocchio::joint_type_prefix.size() + 1] != 'U')) {
        const auto bound = pinocchio_model->getJointLimit(pinocchio_id);
        const auto subspace = std::make_shared<ob::RealVectorStateSpace>(bound.rows());
        auto ob_bounds = ob::RealVectorBounds(bound.rows());
        ASSERT(bound.rows() == 1,
               "Only support simple joint the dim of joint is not 1!");

        lower_joint_limits_.push_back(bound(0, 0));
        upper_joint_limits_.push_back(bound(0, 1));
        ob_bounds.setLow(0, bound(0, 0)), ob_bounds.setHigh(0, bound(0, 1));
        subspace->setBounds(ob_bounds);
        cs_->addSubspace(subspace, 1.0);
        ++dim_cnt;
      } else if (joint_type[pinocchio::joint_type_prefix.size()] == 'R' &&
                 joint_type[pinocchio::joint_type_prefix.size() + 1] == 'U') {
        cs_->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
        lower_joint_limits_.push_back(-PI);
        upper_joint_limits_.push_back(PI);
        ++dim_cnt;
      }
      if (joint_type[pinocchio::joint_type_prefix.size()] == 'R' ||
          joint_type[pinocchio::joint_type_prefix.size()] == 'P') {
        if (joint_type[pinocchio::joint_type_prefix.size()] == 'R' &&
            joint_type[pinocchio::joint_type_prefix.size() + 1] != 'U')
          is_revolute_.push_back(true);
        else
          is_revolute_.push_back(false);
      }
    }
    ASSERT(dim_cnt + ignored_dim_cnt == robot->getQposDim(),
           "Dim of bound is different from dim of qpos: " + std::to_string(dim_cnt) +
               " active dims " + std::to_string(ignored_dim_cnt) +
               " ignored dims not equal to " + std::to_string(dof) + " qpos dim");
    dim_ += dim_cnt;
  }

  state_space_ = cs_;
  compound_si_ = std::make_shared<ob::SpaceInformation>(state_space_);
  si_ = compound_si_;
  valid_checker_ =
      std::make_shared<ValidityCheckerTpl<S>>(world_, si_, false, fixed_joints);
  ss_ = std::make_shared<og::SimpleSetup>(si_);
  ss_->setStateValidityChecker(valid_checker_);
}

template <typename S>
VectorX<S> OMPLPlannerTpl<S>::randomSampleNearby(const VectorX<S> &start_state) const {
  for (int cnt = 0; cnt < 1000; ++cnt) {
    S ratio = static_cast<S>((cnt + 1)) / 1000;
    VectorX<S> new_state = start_state;
    for (size_t i = 0; i < dim_; i++) {
      S r = static_cast<S>(rand()) / RAND_MAX * 2 - 1;
      new_state[i] += (upper_joint_limits_[i] - lower_joint_limits_[i]) * ratio * r;
      if (new_state[i] < lower_joint_limits_[i])
        new_state[i] = lower_joint_limits_[i];
      else if (new_state[i] > upper_joint_limits_[i])
        new_state[i] = upper_joint_limits_[i];
    }
    if (valid_checker_->_isValid(new_state)) {
      print_warning("sampled a new state with a perturbation of ", ratio * 100,
                    "% joint limits.");
      return new_state;
    }
  }
  return start_state;
}

template <typename S>
ob::GoalStatesPtr OMPLPlannerTpl<S>::makeGoalStates(
    const std::vector<VectorX<S>> &goal_states) const {
  auto goals = std::make_shared<ob::GoalStates>(si_);

  int tot_enum_states = 1, tot_goal_state = 0;
  for (size_t i = 0; i < dim_; i++) tot_enum_states *= 3;

  for (size_t ii = 0; ii < goal_states.size(); ii++) {
    for (int i = 0; i < tot_enum_states; i++) {
      std::vector<double> tmp_state;
      int tmp = i;
      bool flag = true;
      for (size_t j = 0; j < dim_; j++) {
        tmp_state.push_back(goal_states[ii](j));
        int dir = tmp % 3;
        tmp /= 3;
        if (dir != 0 && is_revolute_[j] == false) {
          flag = false;
          break;
        }
        if (dir == 1) {
          if (tmp_state[j] - 2 * PI > lower_joint_limits_[j])
            tmp_state[j] -= 2 * PI;
          else {
            flag = false;
            break;
          }
        } else if (dir == 2) {
          if (tmp_state[j] + 2 * PI < upper_joint_limits_[j])
            tmp_state[j] += 2 * PI;
          else {
            flag = false;
            break;
          }
        }
      }
      if (flag) {
        ob::ScopedState<> goal(state_space_);
        goal = tmp_state;
        goals->addState(goal);
        tot_goal_state += 1;
      }
    }
  }
  return goals;
}

template <typename S>
std::pair<std::string, MatrixX<S>> OMPLPlannerTpl<S>::plan(
    const VectorX<S> &start_state, const std::vector<VectorX<S>> &goal_states,
    double time, double range, const FixedJointsTpl<S> &fixed_joints, bool simplify,
    const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)>
        &constraint_function,
    const std::function<void(const VectorXd &, Eigen::Ref<VectorXd>)>
        &constraint_jacobian,
    double constraint_tolerance, bool verbose) {
  if (fixed_joints.size() || last_fixed_joints_.size()) {
    ASSERT(constraint_function == nullptr && constraint_jacobian == nullptr,
           "Cannot do constrained planning with fixed joints");
    buildCompoundStateSpace(fixed_joints);
  } else if (constraint_function != nullptr &&
             constraint_jacobian != nullptr) {  // we have a constraint
    if (last_fixed_joints_.size()) buildConstrainedAmbientStateSpace();
    auto general_constraint = std::make_shared<GeneralConstraint>(
        dim_, constraint_function, constraint_jacobian);
    general_constraint->setTolerance(constraint_tolerance);
    constrained_space_ =
        std::make_shared<ob::ProjectedStateSpace>(ambient_space_, general_constraint);
    state_space_ = constrained_space_;
    constrained_si_ =
        std::make_shared<ob::ConstrainedSpaceInformation>(constrained_space_);
    si_ = constrained_si_;
    valid_checker_ = std::make_shared<ValidityCheckerTpl<S>>(world_, si_, true);
    ss_ = std::make_shared<og::SimpleSetup>(si_);
  }
  last_fixed_joints_ = fixed_joints;

  VectorX<S> reduced_start_state = removeFixedJoints<S>(fixed_joints, start_state);
  std::vector<VectorX<S>> reduced_goal_states;
  for (const auto &goal_state : goal_states)
    reduced_goal_states.push_back(removeFixedJoints<S>(fixed_joints, goal_state));

  ASSERT(reduced_start_state.rows() == reduced_goal_states[0].rows(),
         "Length of start state and goal state should be equal");
  ASSERT(static_cast<size_t>(reduced_start_state.rows()) == dim_,
         "Length of start state and problem dimension should be equal");
  if (verbose == false) ::ompl::msg::noOutputHandler();

  ob::ScopedState<> start(state_space_);
  start = eigen2Vector<S, double>(reduced_start_state);

  bool invalid_start = !valid_checker_->_isValid(reduced_start_state);
  if (invalid_start) {
    print_warning("invalid start state!! (collision)");
    VectorX<S> new_reduced_start_state = randomSampleNearby(reduced_start_state);
    start = eigen2Vector<S, double>(new_reduced_start_state);
  }

  auto goals = makeGoalStates(reduced_goal_states);

  ss_->clear();  // must clear!!! otherwise the planner stitch together the previous
                 // path and the new path
  ss_->setStartState(start);
  ss_->setGoal(goals);

  ob::PlannerPtr planner;
  // RRTConnect
  auto rrt_connect = std::make_shared<og::RRTConnect>(si_);
  if (range > 1E-6) rrt_connect->setRange(range);
  planner = rrt_connect;

  ss_->setPlanner(planner);
  ss_->setup();
  ob::PlannerStatus solved = ss_->solve(time);
  if (solved) {
    if (verbose) print_verbose("Found solution");

    // obtain the path
    auto path = ss_->getSolutionPath();

    // simplify the path if not planning in constrained space
    if (simplify && state_space_ != constrained_space_) _simplifyPath(path);

    size_t len = path.getStateCount();
    MatrixX<S> ret(len + invalid_start, start_state.rows());
    if (verbose) print_verbose("Result size ", len, " ", start_state.rows());
    if (invalid_start) {
      for (auto j = 0; j < start_state.rows(); j++) ret(0, j) = start_state(j);
    }
    for (size_t i = 0; i < len; i++) {
      auto res_i = state2Eigen<S>(path.getState(i), si_.get(),
                                  state_space_ == constrained_space_);
      ASSERT(static_cast<size_t>(res_i.rows()) == dim_,
             "Result dimension is not correct!");
      res_i = addFixedJoints<S>(fixed_joints, res_i);
      for (size_t j = 0; j < static_cast<size_t>(start_state.rows()); j++)
        ret(invalid_start + i, j) = res_i[j];
    }
    return std::make_pair(solved.asString(), ret);
  } else {
    MatrixX<S> ret(0, start_state.rows());
    return std::make_pair(solved.asString(), ret);
  }
}

template <typename S>
void OMPLPlannerTpl<S>::_simplifyPath(og::PathGeometric &path) const {
  // try to simply the path and restore if new path contains collision
  og::PathGeometric backup_path = path;
  og::PathSimplifier simplifer(si_);
  if (!simplifer.simplifyMax(path)) path = backup_path;
}

template <typename S>
MatrixX<S> OMPLPlannerTpl<S>::simplifyPath(const MatrixX<S> &path) const {
  if (si_ == constrained_si_) {  // warning
    print_warning("Current space information is for constrained planning");
    print_warning("doing simplification in non-constrained space");
  }
  og::PathGeometric geo_path(si_);
  for (size_t i = 0; i < static_cast<size_t>(path.rows()); i++) {
    ob::ScopedState<> state(cs_);
    state = eigen2Vector<S, double>(path.row(i));
    geo_path.append(state.get());
  }
  _simplifyPath(geo_path);
  MatrixX<S> ret(geo_path.getStateCount(), dim_);
  for (size_t i = 0; i < geo_path.getStateCount(); i++) {
    // we should only simplify the path when not doing constrained planning
    ret.row(i) = state2Eigen<S>(geo_path.getState(i), si_.get(), false);
  }
  return ret;
}

}  // namespace mplib::planning::ompl
