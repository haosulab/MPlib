#include "ompl_planner.h"

#include "pinocchio_model.h"

#define DEFINE_TEMPLATE_OMPL(DATATYPE)         \
  template class ValidityCheckerTpl<DATATYPE>; \
  template class OMPLPlannerTpl<DATATYPE>;

DEFINE_TEMPLATE_OMPL(double)

#define PI 3.14159265359

bool is_fixed_joint(const FixedJoints &fixed_joints, size_t articulation_idx,
                    size_t joint_idx) {
  for (const auto &fixed_joint : fixed_joints) {
    if (fixed_joint.articulation_idx == articulation_idx &&
        fixed_joint.joint_idx == joint_idx)
      return true;
  }
  return false;
}

Eigen::VectorXd remove_fixed_joints(const FixedJoints &fixed_joints,
                                    const Eigen::VectorXd &state) {
  Eigen::VectorXd ret(state.rows() - fixed_joints.size());
  size_t cnt = 0;
  for (auto i = 0; i < state.rows(); i++) {
    if (is_fixed_joint(fixed_joints, 0, i))
      continue;  // TODO[xinsong] only support one robot rn
    ret[cnt++] = state[i];
  }
  return ret;
}

Eigen::VectorXd add_fixed_joints(const FixedJoints &fixed_joints,
                                 const Eigen::VectorXd &state) {
  auto fixed_itr = fixed_joints.begin();
  Eigen::VectorXd ret(fixed_joints.size() + state.rows());
  size_t j = 0;
  for (auto i = 0; i < ret.rows(); i++) {
    if (is_fixed_joint(fixed_joints, 0, i)) {
      ret[i] = fixed_itr->value;
      ++fixed_itr;
    } else {
      ret[i] = state[j++];
    }
  }
  return ret;
}

template <typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::build_compound_state_space(
    const FixedJoints &fixed_joints) {
  cs = std::make_shared<CompoundStateSpace>();
  dim = 0;
  lower_joint_limits.clear();
  upper_joint_limits.clear();
  is_revolute.clear();
  const std::string joint_prefix = "JointModel";
  auto robots = world->getArticulations();
  for (size_t robot_idx = 0; robot_idx < robots.size(); ++robot_idx) {
    auto robot = robots[robot_idx];
    size_t dim_i = 0;
    auto model = robot->getPinocchioModel();
    auto joint_types = model.getJointTypes();
    size_t d = robot->getQposDim();  // TODO!!! only construct for move group joints
    auto indices = robot->getMoveGroupJointIndices();
    ASSERT(d == indices.size(), "QposDim != size of the movegroup joints");
    size_t ignored_dim = 0;
    for (size_t i = 0; i < d; i++) {
      auto id = indices[i];
      // if we have already fixed this joint, we don't need to add it to the state space
      if (is_fixed_joint(fixed_joints, robot_idx, i)) {
        ++ignored_dim;
        continue;
      }

      auto joint_type = joint_types[id];
      if (joint_type[joint_prefix.size()] == 'P' ||
          (joint_type[joint_prefix.size()] == 'R' &&
           joint_type[joint_prefix.size() + 1] != 'U'))  // PRISMATIC and REVOLUTE
      {
        auto bound = model.getJointLimit(id);
        auto subspcae = std::make_shared<ob::RealVectorStateSpace>(bound.rows());
        auto ob_bounds = ob::RealVectorBounds(bound.rows());
        ASSERT(bound.rows() == 1,
               "Only support simple joint the dim of joint is not 1!");
        lower_joint_limits.push_back(bound(0, 0));
        upper_joint_limits.push_back(bound(0, 1));
        ob_bounds.setLow(0, bound(0, 0)), ob_bounds.setHigh(0, bound(0, 1));
        ++dim_i;
        subspcae->setBounds(ob_bounds);
        cs->addSubspace(subspcae, 1.0);
      } else if (joint_type[joint_prefix.size()] == 'R' &&
                 joint_type[joint_prefix.size() + 1] == 'U') {
        cs->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
        lower_joint_limits.push_back(-PI);
        upper_joint_limits.push_back(PI);
        ++dim_i;
      }
      if (joint_type[joint_prefix.size()] == 'R' ||
          joint_type[joint_prefix.size()] == 'P') {
        if (joint_type[joint_prefix.size()] == 'R' &&
            joint_type[joint_prefix.size() + 1] != 'U')
          is_revolute.push_back(true);
        else
          is_revolute.push_back(false);
      }
    }
    ASSERT(dim_i + ignored_dim == robot->getQposDim(),
           "Dim of bound is different from dim of qpos: " + std::to_string(dim_i) +
               " active dims " + std::to_string(ignored_dim) +
               " ignored dims not equal to " + std::to_string(d) + " qpos dim");
    dim += dim_i;
  }

  state_space = cs;
  p_compound_si = std::make_shared<SpaceInformation>(state_space);
  si = p_compound_si;
  valid_checker = std::make_shared<ValidityChecker>(world, si, false, fixed_joints);
  ss = std::make_shared<ompl::geometric::SimpleSetup>(si);
  ss->setStateValidityChecker(valid_checker);
}

template <typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::build_constrained_ambient_state_space(void) {
  lower_joint_limits.clear();
  upper_joint_limits.clear();
  is_revolute.clear();
  const std::string joint_prefix = "JointModel";
  ASSERT(world->getArticulations().size() == 1,
         "only support one robot for constrained planning");
  auto robot = world->getArticulations()[0];
  // we assume each joint has only one DoF
  dim = robot->getQposDim();  // this getQposDim() is really getMoveGroupQposDim()
  p_ambient_space = std::make_shared<ob::RealVectorStateSpace>(dim);
  ob::RealVectorBounds ambient_space_bounds(dim);
  auto pinocchio_model = robot->getPinocchioModel();
  auto joint_types = pinocchio_model.getJointTypes();
  auto indices = robot->getMoveGroupJointIndices();
  ASSERT(dim == indices.size(), "movegroupQposDim != size of the movegroup joints");
  for (size_t i = 0; i < dim; i++) {
    auto pinocchio_id = indices[i];  // joint_id to pinocchio_id
    auto joint_type = joint_types[pinocchio_id];
    // P stands for PRISMATIC, R stands for REVOLUTE, U stands for UNBOUNDED (continuous
    // joint)
    if (joint_type[joint_prefix.size()] == 'R') {
      ASSERT(joint_type[joint_prefix.size() + 1] != 'U',
             "Do not support continuous revolute joint for constrained planning");
    }
    if (joint_type[joint_prefix.size()] == 'P' ||
        joint_type[joint_prefix.size()] == 'R') {
      auto bound = pinocchio_model.getJointLimit(pinocchio_id);
      ASSERT(bound.rows() == 1,
             "Only support simple joint with dim of joint being one");
      auto lower_joint_limit = bound(0, 0), upper_joint_limit = bound(0, 1);
      ambient_space_bounds.setLow(i, lower_joint_limit);
      ambient_space_bounds.setHigh(i, upper_joint_limit);
    }
  }
  p_ambient_space->setBounds(ambient_space_bounds);
}

template <typename DATATYPE>
OMPLPlannerTpl<DATATYPE>::OMPLPlannerTpl(const PlanningWorldTpl_ptr<DATATYPE> &world,
                                         int robot_idx)
    : world(world) {
  build_constrained_ambient_state_space();  // default is constrained space
  build_compound_state_space();             // builds the ambient space for later
}

template <typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>
OMPLPlannerTpl<DATATYPE>::random_sample_nearby(const VectorX &start_state) {
  for (int cnt = 0; cnt < 1000; ++cnt) {
    DATATYPE ratio = (DATATYPE)(cnt + 1) / 1000;
    VectorX new_state = start_state;
    for (size_t i = 0; i < dim; i++) {
      DATATYPE r = (DATATYPE)rand() / RAND_MAX * 2 - 1;
      new_state[i] += (upper_joint_limits[i] - lower_joint_limits[i]) * ratio * r;
      if (new_state[i] < lower_joint_limits[i])
        new_state[i] = lower_joint_limits[i];
      else if (new_state[i] > upper_joint_limits[i])
        new_state[i] = upper_joint_limits[i];
    }
    if (valid_checker->_isValid(new_state)) {
      print_warning("sampled a new state with a perturbation of ", ratio * 100,
                    "% joint limits.");
      return new_state;
    }
  }
  return start_state;
}

template <typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::_simplify_path(og::PathGeometric &path) {
  // try to simply the path and restore if new path contains collision
  og::PathSimplifier simplifer(si);
  og::PathGeometric backup_path = path;
  if (!simplifer.simplifyMax(path)) path = backup_path;
}

template <typename DATATYPE>
Eigen::MatrixXd OMPLPlannerTpl<DATATYPE>::simplify_path(Eigen::MatrixXd &path) {
  if (si == p_constrained_si) {  // warning
    print_warning("Current space information is for constrained planning");
    print_warning("doing simplification in non-constrained space");
  }
  og::PathGeometric geo_path(si);
  for (size_t i = 0; i < path.rows(); i++) {
    ob::ScopedState<> state(cs);
    state = eigen2vector<DATATYPE, double>(path.row(i));
    geo_path.append(state.get());
  }
  _simplify_path(geo_path);
  Eigen::MatrixXd ret(geo_path.getStateCount(), dim);
  for (size_t i = 0; i < geo_path.getStateCount(); i++) {
    // we should only simplify the path when not doing constrained planning
    ret.row(i) = state2eigen<DATATYPE>(geo_path.getState(i), si.get(), false);
  }
  return ret;
}

template <typename DATATYPE>
std::shared_ptr<ob::GoalStates> OMPLPlannerTpl<DATATYPE>::make_goal_states(
    const std::vector<VectorX> &goal_states) {
  auto goals = std::make_shared<ob::GoalStates>(si);

  int tot_enum_states = 1, tot_goal_state = 0;
  for (size_t i = 0; i < dim; i++) tot_enum_states *= 3;

  for (int ii = 0; ii < goal_states.size(); ii++) {
    for (int i = 0; i < tot_enum_states; i++) {
      std::vector<double> tmp_state;
      int tmp = i;
      bool flag = true;
      for (size_t j = 0; j < dim; j++) {
        tmp_state.push_back(goal_states[ii](j));
        int dir = tmp % 3;
        tmp /= 3;
        if (dir != 0 && is_revolute[j] == false) {
          flag = false;
          break;
        }
        if (dir == 1) {
          if (tmp_state[j] - 2 * PI > lower_joint_limits[j])
            tmp_state[j] -= 2 * PI;
          else {
            flag = false;
            break;
          }
        } else if (dir == 2) {
          if (tmp_state[j] + 2 * PI < upper_joint_limits[j])
            tmp_state[j] += 2 * PI;
          else {
            flag = false;
            break;
          }
        }
      }
      if (flag) {
        ob::ScopedState<> goal(state_space);
        goal = tmp_state;
        goals->addState(goal);
        tot_goal_state += 1;
      }
    }
  }
  return goals;
}

template <typename DATATYPE>
std::pair<std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
OMPLPlannerTpl<DATATYPE>::plan(
    const VectorX &start_state, const std::vector<VectorX> &goal_states,
    const std::string &planner_name, const double &time, const double &range,
    const bool verbose, const FixedJoints &fixed_joints, const bool no_simplification,
    std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)>
        &constraint_function,
    std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)>
        &constraint_jacobian,
    double constraint_tolerance) {
  if (fixed_joints.size() || last_fixed_joints.size()) {
    ASSERT(constraint_function == nullptr && constraint_jacobian == nullptr,
           "Cannot do constrained planning with fixed joints");
    build_compound_state_space(fixed_joints);
  } else if (constraint_function != nullptr &&
             constraint_jacobian != nullptr) {  // we have a constraint
    if (last_fixed_joints.size()) build_constrained_ambient_state_space();
    auto general_constraint = std::make_shared<GeneralConstraint>(
        dim, constraint_function, constraint_jacobian);
    general_constraint->setTolerance(constraint_tolerance);
    p_constrained_space =
        std::make_shared<ob::ProjectedStateSpace>(p_ambient_space, general_constraint);
    state_space = p_constrained_space;
    p_constrained_si =
        std::make_shared<ob::ConstrainedSpaceInformation>(p_constrained_space);
    si = p_constrained_si;
    valid_checker = std::make_shared<ValidityChecker>(world, si, true);
    ss = std::make_shared<ompl::geometric::SimpleSetup>(si);
  }
  last_fixed_joints = fixed_joints;

  Eigen::VectorXd reduced_start_state = remove_fixed_joints(fixed_joints, start_state);
  std::vector<Eigen::VectorXd> reduced_goal_states;
  for (auto &goal_state : goal_states)
    reduced_goal_states.push_back(remove_fixed_joints(fixed_joints, goal_state));

  ASSERT(reduced_start_state.rows() == reduced_goal_states[0].rows(),
         "Length of start state and goal state should be equal");
  ASSERT(reduced_start_state.rows() == dim,
         "Length of start state and problem dimension should be equal");
  if (verbose == false) ompl::msg::noOutputHandler();

  ob::ScopedState<> start(state_space);
  start = eigen2vector<DATATYPE, double>(reduced_start_state);

  bool invalid_start = !valid_checker->_isValid(reduced_start_state);
  if (invalid_start) {
    print_warning("invalid start state!! (collision)");
    VectorX new_reduced_start_state = random_sample_nearby(reduced_start_state);
    start = eigen2vector<DATATYPE, double>(new_reduced_start_state);
  }

  auto goals = make_goal_states(reduced_goal_states);

  ss->clear();  // must clear!!! otherwise the planner stitch together the previous path
                // and the new path
  ss->setStartState(start);
  ss->setGoal(goals);

  ob::PlannerPtr planner;
  if (planner_name == "RRTConnect") {
    auto rrt_connect = std::make_shared<og::RRTConnect>(si);
    if (range > 1E-6) rrt_connect->setRange(range);
    planner = rrt_connect;
  } else if (planner_name == "RRT*") {
    auto rrt_star = std::make_shared<og::RRTstar>(si);
    if (range > 1E-6) rrt_star->setRange(range);
    planner = rrt_star;
  } else {
    throw std::runtime_error(
        "Planner Not implemented, please choose from {RRTConnect, RRT*}");
  }

  ss->setPlanner(planner);
  ss->setup();
  ob::PlannerStatus solved = ss->solve(time);
  if (solved) {
    if (verbose) print_verbose("Found solution");

    // obtain the path
    auto path = ss->getSolutionPath();

    // simplify the path if not planning in constrained space
    if (!no_simplification && state_space != p_constrained_space) _simplify_path(path);

    size_t len = path.getStateCount();
    Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(len + invalid_start,
                                                                start_state.rows());
    if (verbose) print_verbose("Result size ", len, " ", start_state.rows());
    if (invalid_start) {
      for (auto j = 0; j < start_state.rows(); j++) ret(0, j) = start_state(j);
    }
    for (size_t i = 0; i < len; i++) {
      auto res_i = state2eigen<DATATYPE>(path.getState(i), si.get(),
                                         state_space == p_constrained_space);
      ASSERT(res_i.rows() == dim, "Result dimension is not correct!");
      res_i = add_fixed_joints(fixed_joints, res_i);
      for (size_t j = 0; j < start_state.rows(); j++)
        ret(invalid_start + i, j) = res_i[j];
    }
    return std::make_pair(solved.asString(), ret);
  } else {
    Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(0, start_state.rows());
    return std::make_pair(solved.asString(), ret);
  }
}
