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
        auto &model = robot->getPinocchioModel();
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
            }
            is_revolute.push_back(joint_type[joint_prefix.size()] == 'R');
        }
        ASSERT(dim_i == robot->getQposDim(), "Dim of bound is different from dim of qpos " +  std::to_string(dim_i) + " " + std::to_string(robot->getQposDim()));
        dim += dim_i;
    }
    p_ambient_space = std::make_shared<ob::RealVectorStateSpace>(dim);
    ob::RealVectorBounds ambient_space_bounds(dim);
    for (size_t i = 0; i < dim; i++) {
        ambient_space_bounds.setLow(i, lower_joint_limits[i]);
        ambient_space_bounds.setHigh(i, upper_joint_limits[i]);
    }
    p_ambient_space->setBounds(ambient_space_bounds); 
}

template<typename DATATYPE>
void OMPLPlannerTpl<DATATYPE>::build_constrained_state_space(void) {
    std::string const joint_prefix = "JointModel";
    ASSERT (world->getArticulations().size() == 1, "only support one robot for constrained planning");
    auto robot = world->getArticulations()[0];
    // we assume each joint has only one DoF
    dim = robot->getQposDim();  // this getQposDim() is really getMoveGroupQposDim()
    p_ambient_space = std::make_shared<ob::RealVectorStateSpace>(dim);
    ob::RealVectorBounds ambient_space_bounds(dim);
    auto &pinocchio_model = robot->getPinocchioModel();
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
            lower_joint_limits.push_back(lower_joint_limit);
            upper_joint_limits.push_back(upper_joint_limit);
            ambient_space_bounds.setLow(i, lower_joint_limit);
            ambient_space_bounds.setHigh(i, upper_joint_limit);
            is_revolute.push_back(joint_type[joint_prefix.size()] == 'R');
        }
    }
    p_ambient_space->setBounds(ambient_space_bounds);
}

// template<typename DATATYPE>
// OMPLPlannerTpl<DATATYPE>::OMPLPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world):world(world) {
//     build_state_space();
//     si = std::make_shared<SpaceInformation>(cs);
//     valid_checker = std::make_shared<ValidityChecker>(world, si);
//     si->setStateValidityChecker(valid_checker);

//     pdef = std::make_shared<ob::ProblemDefinition>(si);
// }

template<typename DATATYPE>
OMPLPlannerTpl<DATATYPE>::OMPLPlannerTpl(const PlanningWorldTpl_ptr<DATATYPE> &world,
                                         bool constrained_problem, int robot_idx) : world(world) {
    if (constrained_problem) {
        build_constrained_state_space();
        Eigen::Vector3d v(0, 0, -1);
        auto level_constraint = std::make_shared<LevelConstraint>(world->getArticulations()[robot_idx], v);
        p_constrained_space = std::make_shared<ob::ProjectedStateSpace>(p_ambient_space, level_constraint);
        si = std::make_shared<ob::ConstrainedSpaceInformation>(p_constrained_space);
        state_space = p_constrained_space;
    } else {
        build_state_space();
        si = std::make_shared<SpaceInformation>(p_ambient_space);
        state_space = p_ambient_space;
    }
    valid_checker = std::make_shared<ValidityChecker>(world, si);
    ss = std::make_shared<ompl::geometric::SimpleSetup>(si);
    ss->setStateValidityChecker(valid_checker);
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

// template<typename DATATYPE>
// std::pair<std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
// OMPLPlannerTpl<DATATYPE>::plan(VectorX const &start_state, std::vector<VectorX> const &goal_states, const std::string &planner_name,
//                                 const double &time, const double& range, const bool& verbose) {
//     ASSERT(start_state.rows() == goal_states[0].rows(),
//         "Length of start state " + std::to_string(start_state.rows()) +
//         " =/= length of goal state " + std::to_string(goal_states[0].rows()));
//     ASSERT(start_state.rows() == dim,
//         "Length of start state " + std::to_string(start_state.rows()) +
//         " =/= dimension of sample space " + std::to_string(dim) +
//         ". Please only provide the move group joints.");
//     if (verbose == false)
//         ompl::msg::noOutputHandler();

//     std::cout << "planning" << std::endl;
//     ob::ScopedState<> start(state_space);
//     //TODO[xinsong] change back to general one
//     std::cout << "planning" << std::endl;
//     start->as<ob::ConstrainedStateSpace::StateType>()->copy(start_state);

//     std::cout << "planning" << std::endl;
//     bool invalid_start = !valid_checker->_isValid(start_state);
//     if (invalid_start) {
//         std::cout << "invalid start state!! (collision)" << std::endl;
//         VectorX new_start_state = random_sample_nearby(start_state);
//         //TODO[xinsong] change back to general one
//         start->as<ob::ConstrainedStateSpace::StateType>()->copy(start_state);
//     }

//     std::cout << "planning" << std::endl;
//     auto goals = std::make_shared<ob::GoalStates>(si);

//     int tot_enum_states = 1, tot_goal_state = 0;
//     for (int i = 0; i < dim; i++) 
//         tot_enum_states *= 3;

//     for (int ii = 0; ii < goal_states.size(); ii++)
//         for (int i = 0; i < tot_enum_states; i++) {
//             std::vector<double> tmp_state;
//             int tmp = i;
//             bool flag = true;
//             for (int j = 0; j < dim; j++) {
//                 tmp_state.push_back(goal_states[ii](j));
//                 int dir = tmp % 3;
//                 tmp /= 3;
//                 if (dir != 0 && is_revolute[j] == false) {
//                     flag = false;
//                     break;
//                 }
//                 if (dir == 1) {
//                     if (tmp_state[j] - 2 * PI > lower_joint_limits[j]) 
//                         tmp_state[j] -= 2 * PI;
//                     else {
//                         flag = false;
//                         break;
//                     }
//                 }
//                 else if (dir == 2) {
//                     if (tmp_state[j] + 2 * PI < upper_joint_limits[j]) 
//                         tmp_state[j] += 2 * PI;
//                     else {
//                         flag = false;
//                         break;
//                     }                
//                 }
//             }
//             if (flag) {
//                 ob::ScopedState<> goal(state_space);
//                 goal = tmp_state; 
//                 goals->addState(goal);
//                 tot_goal_state += 1;
//             }
//         }
//     if (verbose)
//         std::cout << "number of goal state: " << tot_goal_state << std::endl;

//     std::cout << "planning" << std::endl;
//     pdef->clearStartStates();
//     std::cout << "planning" << std::endl;
//     pdef->clearGoal();
//     std::cout << "planning" << std::endl;
//     pdef->clearSolutionPaths();
//     std::cout << "planning" << std::endl;
//     pdef->clearSolutionNonExistenceProof();
//     //pdef->setStartAndGoalStates(start, goal);
//     std::cout << "planning" << std::endl;
//     pdef->setGoal(goals);
//     std::cout << "planning" << std::endl;
//     pdef->addStartState(start);
//     std::cout << "planning" << std::endl;
//     ob::PlannerPtr planner;
//     if (planner_name == "RRTConnect") {
//         auto rrt_connect = std::make_shared<og::RRTConnect>(si);
//         if (range > 1E-6)
//             rrt_connect->setRange(range);
//         planner = rrt_connect;
//     } else if (planner_name == "RRT*") {
//         auto rrt_star = std::make_shared<og::RRTstar>(si);
//         if (range > 1E-6)
//             rrt_star->setRange(range);
//         planner = rrt_star;
//     } else {
//         throw std::runtime_error("Planner Not implemented, please choose from {RRTConnect, RRT*}");
//     }

//     std::cout << "planning" << std::endl;
//     planner->setProblemDefinition(pdef);
//     std::cout << "planning" << std::endl;
//     planner->setup();
//     std::cout << "planning" << std::endl;
//     if (verbose)
//         std::cout << "OMPL setup" << std::endl;
//     ob::PlannerStatus solved = planner->ob::Planner::solve(time);
//     std::cout << "planning" << std::endl;
//     if (solved) {
//         if (verbose) std::cout << "Solved!" << std::endl;

//         // obtain the path
//         ob::PathPtr path = pdef->getSolutionPath();
//         auto geoPathPtr = std::dynamic_pointer_cast<og::PathGeometric>(path);

//         // try to simply the path and restore if new path contains collision
//         auto geoPathBackup = *geoPathPtr;
//         og::PathSimplifier simplifer(si);
//         if (!simplifer.simplifyMax(*geoPathPtr)) *geoPathPtr = geoPathBackup;

//         if (verbose) {
//             std::cout << "Path length before simplification: " << geoPathBackup.getStateCount() << std::endl;
//             std::cout << "Path length after simplification: " << geoPathPtr->getStateCount() << std::endl;
//         }

//         size_t len = geoPathPtr->getStateCount();
//         Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(len + invalid_start, dim);
//         if (verbose)
//             std::cout << "Result size " << len << " " << dim << std::endl;
//         if (invalid_start) {
//             for (int j = 0; j < dim; j++)
//                 ret(0, j) = start_state(j);
//         }
//         for (size_t i = 0; i < len; i++) {
//             auto res_i = state2eigen<DATATYPE>(geoPathPtr->getState(i), si.get());
//             //std::cout << "Size_i " << res_i.rows() << std::endl;
//             ASSERT(res_i.rows() == dim, "Result dimension is not correct!");
//             for (size_t j = 0; j < dim; j++)
//                 ret(invalid_start + i, j) = res_i[j];
//         }
//         return std::make_pair(solved.asString(), ret);
//     }
//     else {
//         Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic> ret(0, dim);
//         return std::make_pair(solved.asString(), ret);
//     }
// }

template<typename DATATYPE>
std::pair<std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
OMPLPlannerTpl<DATATYPE>::plan(VectorX const &start_state, std::vector<VectorX> const &goal_states, const std::string &planner_name,
                                const double &time, const double& range, const bool& verbose) {
    ASSERT(start_state.rows() == goal_states[0].rows(),
        "Length of start state " + std::to_string(start_state.rows()) +
        " =/= length of goal state " + std::to_string(goal_states[0].rows()));
    ASSERT(start_state.rows() == dim,
        "Length of start state " + std::to_string(start_state.rows()) +
        " =/= dimension of sample space " + std::to_string(dim) +
        ". Please only provide the move group joints.");
    if (verbose == false)
        ompl::msg::noOutputHandler();

    ob::ScopedState<> start(state_space);
    start = eigen2vector<DATATYPE, double>(start_state);

    bool invalid_start = !valid_checker->_isValid(start_state);
    if (invalid_start) {
        std::cout << "invalid start state!! (collision)" << std::endl;
        VectorX new_start_state = random_sample_nearby(start_state);
        start = eigen2vector<DATATYPE, double>(new_start_state);
    }

    auto goals = std::make_shared<ob::GoalStates>(si);

    int tot_enum_states = 1, tot_goal_state = 0;
    for (int i = 0; i < dim; i++) 
        tot_enum_states *= 3;

    for (int ii = 0; ii < goal_states.size(); ii++)
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

    std::cout << "planning" << std::endl;
    ss->setStartState(start);
    std::cout << "planning" << std::endl;
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

    std::cout << "planner set" << std::endl;
    ss->setPlanner(planner);

    std::cout << "planning" << std::endl;
    ss->setup();
    std::cout << "setup" << std::endl;
    ob::PlannerStatus solved = ss->solve(time);
    std::cerr << "solved" << std::endl;
    if (solved) {
        if (verbose) std::cout << "Solved!" << std::endl;

        // obtain the path
        auto path = ss->getSolutionPath();

        // try to simply the path and restore if new path contains collision
        auto pathBackup = path;
        og::PathSimplifier simplifer(si);
        if (!simplifer.simplifyMax(path)) path = pathBackup;

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
            auto res_i = state2eigen<DATATYPE>(path.getState(i), si.get());
            //std::cout << "Size_i " << res_i.rows() << std::endl;
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
