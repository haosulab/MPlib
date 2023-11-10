#include "ompl_planner.h"
#include "pinocchio_model.h"


#define DEFINE_TEMPLATE_OMPL(DATATYPE) template class ValidityCheckerTpl<DATATYPE>; template class OMPLPlannerTpl<DATATYPE>;

DEFINE_TEMPLATE_OMPL(double)

#define PI 3.14159265359

bool is_fixed_joint(const FixedJoints &fixed_joints, size_t articulation_idx, size_t joint_idx) {
    for (const auto &fixed_joint: fixed_joints) {
        if (fixed_joint.articulation_idx == articulation_idx && fixed_joint.joint_idx == joint_idx)
            return true;
    }
    return false;
}

Eigen::VectorXd remove_fixed_joints(const FixedJoints &fixed_joints, Eigen::VectorXd const &state) {
    Eigen::VectorXd ret(state.rows() - fixed_joints.size());
    size_t cnt = 0;
    for (auto i = 0; i < state.rows(); i++) {
        if (is_fixed_joint(fixed_joints, 0, i)) continue;  //TODO[xinsong] only support one robot rn
        ret[cnt++] = state[i];
    }
    return ret;
}

Eigen::VectorXd add_fixed_joints(const FixedJoints &fixed_joints, Eigen::VectorXd const &state) {
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

template<typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::build_planning_config(FixedJoints const &fixed_joints) {
    PlanningConfig planning_config;
    planning_config.cs = std::make_shared<CompoundStateSpace>();
    planning_config.dim = 0;
    std::string const joint_prefix = "JointModel";
    auto robots = world->getArticulations();
    for (size_t robot_idx = 0; robot_idx < robots.size(); ++robot_idx) {
        auto robot = robots[robot_idx];
        size_t dim_i = 0;
        auto model = robot->getPinocchioModel();
        auto joint_types = model.getJointTypes();
        size_t d = robot->getQposDim(); // TODO!!! only construct for move group joints
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
            if (joint_type[joint_prefix.size()] == 'P' || (joint_type[joint_prefix.size()] == 'R' &&
                                                        joint_type[joint_prefix.size() + 1] != 'U'))// PRISMATIC and REVOLUTE
            {
                auto bound = model.getJointLimit(id);
                auto subspcae = std::make_shared<ob::RealVectorStateSpace>(bound.rows());
                auto ob_bounds = ob::RealVectorBounds(bound.rows());
                ASSERT(bound.rows() == 1, "Only support simple joint the dim of joint is not 1!");
                planning_config.lower_joint_limits.push_back(bound(0, 0));
                planning_config.upper_joint_limits.push_back(bound(0, 1));
                ob_bounds.setLow(0, bound(0, 0)), ob_bounds.setHigh(0, bound(0, 1));
                ++dim_i;
                subspcae->setBounds(ob_bounds);
                planning_config.cs->addSubspace(subspcae, 1.0);
            } else if (joint_type[joint_prefix.size()] == 'R' && joint_type[joint_prefix.size() + 1] == 'U') {
                planning_config.cs->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
                planning_config.lower_joint_limits.push_back(-PI);
                planning_config.upper_joint_limits.push_back(PI);
                ++dim_i;
            }
            if (joint_type[joint_prefix.size()] == 'R' || joint_type[joint_prefix.size()] == 'P') {
                if (joint_type[joint_prefix.size()] == 'R' && joint_type[joint_prefix.size() + 1] != 'U')
                    planning_config.is_revolute.push_back(true);
                else
                    planning_config.is_revolute.push_back(false);
            }
        }
        ASSERT(dim_i + ignored_dim == robot->getQposDim(), "Dim of bound is different from dim of qpos: "
               + std::to_string(dim_i) + " active dims " + std::to_string(ignored_dim) + " ignored dims not equal to "
               + std::to_string(d) + " qpos dim");
        planning_config.dim += dim_i;
    }

    planning_config.si = std::make_shared<SpaceInformation>(planning_config.cs);
    planning_config.valid_checker = std::make_shared<ValidityChecker>(world, planning_config.si, fixed_joints);
    planning_config.si->setStateValidityChecker(planning_config.valid_checker);
    planning_config.pdef = std::make_shared<ob::ProblemDefinition>(planning_config.si);
    planning_configs[fixed_joints] = planning_config;
}

template<typename DATATYPE>
OMPLPlannerTpl<DATATYPE>::OMPLPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world) : world(world) {
    build_planning_config(FixedJoints());
}

template<typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>
OMPLPlannerTpl<DATATYPE>::random_sample_nearby(VectorX const &start_state, PlanningConfig &planning_config) {
    for (int cnt = 0; cnt < 1000; ++cnt) {
        DATATYPE ratio = (DATATYPE) (cnt + 1) / 1000;
        VectorX new_state = start_state;
        for (size_t i = 0; i < planning_config.dim; i++) {
            DATATYPE r = (DATATYPE) rand() / RAND_MAX * 2 - 1;
            new_state[i] += (planning_config.upper_joint_limits[i] - planning_config.lower_joint_limits[i]) * ratio * r;
            if (new_state[i] < planning_config.lower_joint_limits[i])
                new_state[i] = planning_config.lower_joint_limits[i];
            else if (new_state[i] > planning_config.upper_joint_limits[i])
                new_state[i] = planning_config.upper_joint_limits[i];
        }
        if (planning_config.valid_checker->_isValid(new_state)) {
            std::cout << "successfully sampled a new state with a perturbation of "
                      << ratio * 100 << "% joint limits." << std::endl;
            return new_state;
        }
    }
    return start_state;
}

template<typename DATATYPE>
std::pair<std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
OMPLPlannerTpl<DATATYPE>::plan(const VectorX &start_state,
                               const std::vector<VectorX> &goal_states,
                               const std::string &planner_name,
                               const double &time,
                               const double& range,
                               const bool& verbose,
                               const FixedJoints &fixed_joints) {
    if (!planning_configs.count(fixed_joints)) build_planning_config(fixed_joints);
    else planning_configs[fixed_joints].valid_checker->update_fixed_joints(fixed_joints);  // update collision checker
    auto &planning_config = planning_configs[fixed_joints];

    Eigen::VectorXd reduced_start_state = remove_fixed_joints(fixed_joints, start_state);
    std::vector<Eigen::VectorXd> reduced_goal_states;
    for (auto &goal_state: goal_states) reduced_goal_states.push_back(remove_fixed_joints(fixed_joints, goal_state));

    ASSERT(reduced_start_state.rows() == reduced_goal_states[0].rows(),
           "Length of start state and goal state should be equal");
    ASSERT(reduced_start_state.rows() == planning_config.dim,
           "Length of start state and problem dimension should be equal");
    if (verbose == false)
        ompl::msg::noOutputHandler();

    ob::ScopedState<> start(planning_config.cs);
    start = eigen2vector<DATATYPE, double>(reduced_start_state);

    bool invalid_start = !planning_config.valid_checker->_isValid(reduced_start_state);
    if (invalid_start) {
        std::cout << "invalid start state!! (collision)" << std::endl;
        VectorX new_reduced_start_state = random_sample_nearby(reduced_start_state, planning_config);
        start = eigen2vector<DATATYPE, double>(new_reduced_start_state);
    }

    auto goals = std::make_shared<ob::GoalStates>(planning_config.si);

    int tot_enum_states = 1, tot_goal_state = 0;
    for (size_t i = 0; i < planning_config.dim; i++) 
        tot_enum_states *= 3;

    for (size_t ii = 0; ii < reduced_goal_states.size(); ii++) {
        for (int i = 0; i < tot_enum_states; i++) {
            std::vector<double> tmp_state;
            int tmp = i;
            bool flag = true;
            for (size_t j = 0; j < planning_config.dim; j++) {
                tmp_state.push_back(reduced_goal_states[ii](j));
                int dir = tmp % 3;
                tmp /= 3;
                if (dir != 0 && planning_config.is_revolute[j] == false) {
                    flag = false;
                    break;
                }
                if (dir == 1) {
                    if (tmp_state[j] - 2 * PI > planning_config.lower_joint_limits[j]) 
                        tmp_state[j] -= 2 * PI;
                    else {
                        flag = false;
                        break;
                    }
                }
                else if (dir == 2) {
                    if (tmp_state[j] + 2 * PI < planning_config.upper_joint_limits[j]) 
                        tmp_state[j] += 2 * PI;
                    else {
                        flag = false;
                        break;
                    }                
                }
            }
            if (flag) {
                ob::ScopedState<> goal(planning_config.cs);
                goal = tmp_state; 
                goals->addState(goal);
                tot_goal_state += 1;
            }
        }
    }
    if (verbose) std::cout << "number of goal state: " << tot_goal_state << std::endl;

    planning_config.pdef->clearStartStates();
    planning_config.pdef->clearGoal();
    planning_config.pdef->clearSolutionPaths();
    planning_config.pdef->clearSolutionNonExistenceProof();
    //pdef->setStartAndGoalStates(start, goal);
    planning_config.pdef->setGoal(goals);
    planning_config.pdef->addStartState(start);
    ob::PlannerPtr planner;
    if (planner_name == "RRTConnect")
    {
        auto rrt_connect = std::make_shared<og::RRTConnect>(planning_config.si);
        if (range > 1E-6)
            rrt_connect->setRange(range);
        planner = rrt_connect;
    }
    else
        throw std::runtime_error("Planner Not implemented");

    planner->setProblemDefinition(planning_config.pdef);
    planner->setup();
    if (verbose) std::cout << "OMPL setup" << std::endl;
    ob::PlannerStatus solved = planner->ob::Planner::solve(time);
    if (solved) {
        if (verbose) std::cout << "Solved!" << std::endl;
        ob::PathPtr path = planning_config.pdef->getSolutionPath();
        auto geo_path = std::dynamic_pointer_cast<og::PathGeometric>(path);
        size_t len = geo_path->getStateCount();
        Eigen::MatrixXd ret(len + invalid_start, start_state.rows());
        if (verbose) std::cout << "Solved path size " << len << std::endl;
        
        // put the invalid start state back as the original state
        if (invalid_start) {
            for (auto j = 0; j < start_state.rows(); j++)
                ret(0, j) = start_state(j);
        }
        for (size_t i = 0; i < len; i++) {
            auto res_i = state2eigen<DATATYPE>(geo_path->getState(i), planning_config.si.get());
            ASSERT(res_i.rows() == planning_config.dim, "Result dimension is not the same as problem space dimension!");
            res_i = add_fixed_joints(fixed_joints, res_i);
            for (auto j = 0; j < start_state.rows(); j++)
                ret(invalid_start + i, j) = res_i[j];
        }
        return std::make_pair(solved.asString(), ret);
    } else {
        Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(0, start_state.rows());
        return std::make_pair(solved.asString(), ret);
    }
}
