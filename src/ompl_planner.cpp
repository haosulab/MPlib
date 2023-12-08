#include "ompl_planner.h"
#include "pinocchio_model.h"


#define DEFINE_TEMPLATE_OMPL(DATATYPE) template class ValidityCheckerTpl<DATATYPE>; template class OMPLPlannerTpl<DATATYPE>;

DEFINE_TEMPLATE_OMPL(double)

#define PI 3.14159265359

template<typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::build_state_space(void) {
    cs = std::make_shared<CompoundStateSpace>();
    dim = 0;
    std::string const joint_prefix = "JointModel";
    for (auto robot: world->getArticulations()) {
        auto dim_i = 0;
        auto model = robot->getPinocchioModel();
        auto joint_types = model.getJointTypes();
        auto d = robot->getQposDim(); // TODO!!! only construct for move group joints
        auto indices = robot->getMoveGroupJointIndices();
        ASSERT(d == indices.size(), "QposDim != size of the movegroup joints");
        for (size_t i = 0; i < d; i++) {
            auto id = indices[i];
            auto joint_type = joint_types[id];
            if (joint_type[joint_prefix.size()] == 'P' || (joint_type[joint_prefix.size()] == 'R' &&
                                                        joint_type[joint_prefix.size() + 1] != 'U'))// PRISMATIC and REVOLUTE
            {
                auto bound = model.getJointLimit(id);
                auto subspcae = std::make_shared<ob::RealVectorStateSpace>(bound.rows());
                auto ob_bounds = ob::RealVectorBounds(bound.rows());
                dim_i += bound.rows();
                for (size_t j = 0; j < bound.rows(); j++) {
                    lower_joint_limits.push_back(bound(j, 0));
                    upper_joint_limits.push_back(bound(j, 1));
                    ob_bounds.setLow(j, bound(j, 0)), ob_bounds.setHigh(j, bound(j, 1));
                }
                subspcae->setBounds(ob_bounds);
                cs->addSubspace(subspcae, 1.0);
            } else if (joint_type[joint_prefix.size()] == 'R' && joint_type[joint_prefix.size() + 1] == 'U') {
                cs->addSubspace(std::make_shared<ob::SO2StateSpace>(), 1.0);
                lower_joint_limits.push_back(-PI);
                upper_joint_limits.push_back(PI);
                dim_i += 1;
            }
            if (joint_type[joint_prefix.size()] == 'R' || joint_type[joint_prefix.size()] == 'P') {
                if (joint_type[joint_prefix.size()] == 'R' && joint_type[joint_prefix.size() + 1] != 'U')
                    is_revolute.push_back(true);
                else
                    is_revolute.push_back(false);
            }
        }
        ASSERT(dim_i == robot->getQposDim(), "Dim of bound is different from dim of qpos " +  std::to_string(dim_i) + " " + std::to_string(robot->getQposDim()));
        dim += dim_i;
    }
}

template<typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::build_constrained_ambient_state_space(void) {
    std::string const joint_prefix = "JointModel";
    ASSERT (world->getArticulations().size() == 1, "only support one robot for constrained planning");
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
        // P stands for PRISMATIC, R stands for REVOLUTE, U stands for UNBOUNDED (continuous joint)
        if (joint_type[joint_prefix.size()] == 'R') {
            ASSERT(joint_type[joint_prefix.size() + 1] != 'U',
                   "Do not support continuous revolute joint for constrained planning");
        }
        if (joint_type[joint_prefix.size()] == 'P' || joint_type[joint_prefix.size()] == 'R') {
            auto bound = pinocchio_model.getJointLimit(pinocchio_id);
            ASSERT(bound.rows() == 1, "Only support simple joint with dim of joint being one");
            auto lower_joint_limit = bound(0, 0), upper_joint_limit = bound(0, 1);
            ambient_space_bounds.setLow(i, lower_joint_limit);
            ambient_space_bounds.setHigh(i, upper_joint_limit);
        }
    }
    p_ambient_space->setBounds(ambient_space_bounds);
}

template<typename DATATYPE>
OMPLPlannerTpl<DATATYPE>::OMPLPlannerTpl(const PlanningWorldTpl_ptr<DATATYPE> &world, int robot_idx) : world(world) {
    build_state_space();
    build_constrained_ambient_state_space();
    
    p_compound_si = std::make_shared<SpaceInformation>(cs);  // using compound space to begin with
    state_space = cs;
    si = p_compound_si;
    update_ss();
}

template<typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1> OMPLPlannerTpl<DATATYPE>::random_sample_nearby(VectorX const &start_state) {
    int cnt = 0;
    while (true) {
        DATATYPE ratio = (DATATYPE) (cnt + 1) / 1000;
        VectorX new_state = start_state;
        for (int i = 0; i < dim; i++) {
            DATATYPE r = (DATATYPE) rand() / RAND_MAX * 2 - 1;
            new_state[i] += (upper_joint_limits[i] - lower_joint_limits[i]) * ratio * r;
            if (new_state[i] < lower_joint_limits[i])
                new_state[i] = lower_joint_limits[i];
            else if (new_state[i] > upper_joint_limits[i])
                new_state[i] = upper_joint_limits[i];
        }
        if (valid_checker->_isValid(new_state)) {
            std::cout << "successfully sampled a new state with a perturbation of " << ratio * 100 << "% joint limits." << std::endl;
            return new_state;
        }
        cnt += 1;
        if (cnt > 1000)
            return start_state;
    }
}

template<typename DATATYPE>
std::shared_ptr<ob::GoalStates> OMPLPlannerTpl<DATATYPE>::make_goal_states(std::vector<VectorX> const &goal_states) {
    auto goals = std::make_shared<ob::GoalStates>(si);

    int tot_enum_states = 1, tot_goal_state = 0;
    for (int i = 0; i < dim; i++) 
        tot_enum_states *= 3;

    for (int ii = 0; ii < goal_states.size(); ii++) {
        for (int i = 0; i < tot_enum_states; i++) {
            std::vector<double> tmp_state;
            int tmp = i;
            bool flag = true;
            for (int j = 0; j < dim; j++) {
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
                }
                else if (dir == 2) {
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

template<typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::update_ss(bool is_rvss) {
    valid_checker = std::make_shared<ValidityChecker>(world, si, is_rvss);
    ss = std::make_shared<ompl::geometric::SimpleSetup>(si);
    ss->setStateValidityChecker(valid_checker);
}

template<typename DATATYPE>
std::pair<std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
OMPLPlannerTpl<DATATYPE>::plan(VectorX const &start_state,
                               std::vector<VectorX> const &goal_states,
                               const std::string &planner_name,
                               const double &time,
                               const double &range,
                               const bool verbose,
                               const Eigen::Vector3d &align_axis,
                               const double &align_angle,
                               const bool no_simplification) {
    ASSERT(start_state.rows() == goal_states[0].rows(),
        "Length of start state " + std::to_string(start_state.rows()) +
        " =/= length of goal state " + std::to_string(goal_states[0].rows()));
    ASSERT(start_state.rows() == dim,
        "Length of start state " + std::to_string(start_state.rows()) +
        " =/= dimension of sample space " + std::to_string(dim) +
        ". Please only provide the move group joints.");
    if (verbose == false)
        ompl::msg::noOutputHandler();

    if (align_axis != Eigen::Vector3d(0,0,0)) {  // we have a constraint
        auto level_constraint = std::make_shared<LevelConstraint>(world->getArticulations()[0],
                                                                  world->getArticulations()[0]->getEEFrameIndex(),
                                                                  align_axis,
                                                                  std::cos(align_angle));
        level_constraint->setTolerance(1e-3);
        p_constrained_space = std::make_shared<ob::ProjectedStateSpace>(p_ambient_space, level_constraint);
        state_space = p_constrained_space;
        p_constrained_si = std::make_shared<ob::ConstrainedSpaceInformation>(p_constrained_space);
        si = p_constrained_si;
        update_ss(true);
    } else {
        state_space = cs;
        if (si != p_compound_si) {
            si = p_compound_si;
            update_ss();
        }
    }

    ob::ScopedState<> start(state_space);
    start = eigen2vector<DATATYPE, double>(start_state);

    bool invalid_start = !valid_checker->_isValid(start_state);
    if (invalid_start) {
        std::cout << "invalid start state!! (collision)" << std::endl;
        VectorX new_start_state = random_sample_nearby(start_state);
        start = eigen2vector<DATATYPE, double>(new_start_state);
    }

    auto goals = make_goal_states(goal_states);

    ss->clear();  // must clear!!! otherwise the planner stitch together the previous path and the new path
    ss->setStartState(start);
    ss->setGoal(goals);

    ob::PlannerPtr planner;
    if (planner_name == "RRTConnect") {
        auto rrt_connect = std::make_shared<og::RRTConnect>(si);
        if (range > 1E-6)
            rrt_connect->setRange(range);
        planner = rrt_connect;
    } else if (planner_name == "RRT*") {
        auto rrt_star = std::make_shared<og::RRTstar>(si);
        if (range > 1E-6)
            rrt_star->setRange(range);
        planner = rrt_star;
    } else {
        throw std::runtime_error("Planner Not implemented, please choose from {RRTConnect, RRT*}");
    }

    ss->setPlanner(planner);
    ss->setup();
    ob::PlannerStatus solved = ss->solve(time);
    if (solved) {
        if (verbose) std::cout << "Solved!" << std::endl;

        // obtain the path
        auto path = ss->getSolutionPath();

        // try to simply the path and restore if new path contains collision
        auto pathBackup = path;
        og::PathSimplifier simplifer(si);
        // do not simplify if the state space is constrained space
        if (no_simplification || state_space == p_constrained_space || !simplifer.simplifyMax(path)) {
            path = pathBackup;
        }

        if (verbose) {
            std::cout << "Path length before simplification: " << pathBackup.getStateCount() << std::endl;
            std::cout << "Path length after simplification: " << path.getStateCount() << std::endl;
        }

        size_t len = path.getStateCount();
        Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(len + invalid_start, dim);
        if (verbose)
            std::cout << "Result size " << len << " " << dim << std::endl;
        if (invalid_start) {
            for (int j = 0; j < dim; j++)
                ret(0, j) = start_state(j);
        }
        for (size_t i = 0; i < len; i++) {
            // the ambient space of the constrained space is a real vector space
            auto res_i = state2eigen<DATATYPE>(path.getState(i), si.get(), state_space == p_constrained_space);
            ASSERT(res_i.rows() == dim, "Result dimension is not correct!");
            for (size_t j = 0; j < dim; j++)
                ret(invalid_start + i, j) = res_i[j];
        }
        return std::make_pair(solved.asString(), ret);
    }
    else {
        Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(0, dim);
        return std::make_pair(solved.asString(), ret);
    }
}
