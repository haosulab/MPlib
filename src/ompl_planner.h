#pragma once

#include <pinocchio/fwd.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include "planning_world.h"

namespace ob = ompl::base;
//namespace oc = ompl::control;
namespace og = ompl::geometric;


template<typename DATATYPE>
std::vector <DATATYPE> state2vector(const ob::State *state_raw, ob::SpaceInformation *const &si_) {
    auto state = state_raw->as<ob::CompoundState>();
    std::vector <DATATYPE> ret;
    auto si = si_->getStateSpace()->as<ob::CompoundStateSpace>();

    for (size_t i = 0; i < si->getSubspaceCount(); i++) {
        auto subspace(si->getSubspace(i));
        size_t n;
        switch (subspace->getType()) {
            case ob::STATE_SPACE_REAL_VECTOR:
                n = subspace->as<ob::RealVectorStateSpace>()->getDimension();
                for (size_t j = 0; j < n; j++)
                    ret.push_back((DATATYPE) (*state)[i]->as<ob::RealVectorStateSpace::StateType>()->values[j]);
                break;
            case ob::STATE_SPACE_SO2:
                ret.push_back((DATATYPE) (*state)[i]->as<ob::SO2StateSpace::StateType>()->value);
                break;
            default:
                throw std::invalid_argument("Unhandled subspace type.");
                break;
        }
    }
    return ret;
}

template<typename IN_TYPE, typename OUT_TYPE>
std::vector <OUT_TYPE> eigen2vector(Eigen::Matrix<IN_TYPE, Eigen::Dynamic, 1> const &x) {
    std::vector <OUT_TYPE> ret;
    for (size_t i = 0; i < x.rows(); i++)
        ret.push_back((OUT_TYPE) x[i]);
    return ret;
}

template<typename IN_TYPE, typename OUT_TYPE>
Eigen::Matrix<OUT_TYPE, Eigen::Dynamic, 1> vector2eigen(std::vector<IN_TYPE> const &x) {
    Eigen::Matrix<OUT_TYPE, Eigen::Dynamic, 1> ret(x.size());
    for (size_t i = 0; i < x.size(); i++)
        ret[i] = (OUT_TYPE)x[i];
    return ret;
}

template<typename DATATYPE>
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1> state2eigen(const ob::State *state_raw, ob::SpaceInformation *const &si_) {
    auto vec_ret = state2vector<DATATYPE>(state_raw, si_);
    /*for (size_t i = 0; i < vec_ret.size(); i++)
        std::cout << vec_ret[i] << " ";
    std::cout << std::endl;
    */
     auto ret = vector2eigen<DATATYPE, DATATYPE>(vec_ret);
    /*std::cout << ret.rows() << " " << ret.cols() << std::endl;
    for (size_t i = 0; i < ret.rows(); i++)
        std::cout << ret[i] << " ";
    std::cout << std::endl;
    */
    return ret;
}

typedef struct FixedJoint {
    size_t articulation_idx;  // which robot in the planning world does the fixed joint belong to?
    size_t joint_idx;  // what is the index of the joint you want it fixed?
    double value;  // what is the value of the fixed joint?  we are actively trying to get rid of the DATATYPE template

    bool operator==(const FixedJoint &other) const {
        return articulation_idx == other.articulation_idx && joint_idx == other.joint_idx;
    }

    bool operator<(const FixedJoint &other) const {
        return articulation_idx < other.articulation_idx ||
               (articulation_idx == other.articulation_idx && joint_idx < other.joint_idx);
    }
} FixedJoint;

typedef std::set<FixedJoint> FixedJoints;

bool is_fixed_joint(const FixedJoints &fixed_joints, size_t articulation_idx, size_t joint_idx);

Eigen::VectorXd remove_fixed_joints(const FixedJoints &fixed_joints, Eigen::VectorXd const &state);

Eigen::VectorXd add_fixed_joints(const FixedJoints &fixed_joints, Eigen::VectorXd const &state);

template<typename DATATYPE>
class ValidityCheckerTpl : public ob::StateValidityChecker {
    typedef Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1> VectorX;
    PlanningWorldTpl_ptr<DATATYPE> world;
    FixedJoints fixed_joints;

public:
    ValidityCheckerTpl(PlanningWorldTpl_ptr<DATATYPE> world,
                       const ob::SpaceInformationPtr &si, const FixedJoints &fixed_joints)
            : ob::StateValidityChecker(si), world(world), fixed_joints(fixed_joints) {}

    bool _isValid(VectorX state) const {
        world->setQposAll(add_fixed_joints(fixed_joints, state));
        return !world->collide();
    }

    bool isValid(const ob::State *state_raw) const {
        //std::cout << "Begin to check state" << std::endl;
        //std::cout << "check " << state2eigen<DATATYPE>(state_raw, si_) << std::endl;
        auto state = state2eigen<DATATYPE>(state_raw, si_);
        return _isValid(state);
    }
};

template<typename DATATYPE>
using ValidityCheckerTpl_ptr = std::shared_ptr<ValidityCheckerTpl<DATATYPE>>;


using ValidityCheckerd_ptr = ValidityCheckerTpl_ptr<double>;
using ValidityCheckerf_ptr = ValidityCheckerTpl_ptr<float>;
using ValidityCheckerd = ValidityCheckerTpl<double>;
using ValidityCheckerf = ValidityCheckerTpl<float>;


template<typename DATATYPE>
class OMPLPlannerTpl {
    typedef std::shared_ptr <ob::CompoundStateSpace> CompoundStateSpace_ptr;
    typedef std::shared_ptr <ob::SpaceInformation> SpaceInformation_ptr;
    typedef std::shared_ptr <ob::ProblemDefinition> ProblemDefinition_ptr;

    typedef ob::CompoundStateSpace CompoundStateSpace;
    typedef ob::SpaceInformation SpaceInformation;
    typedef ob::ProblemDefinition ProblemDefinition;
    using ValidityChecker = ValidityCheckerTpl<DATATYPE>;
    using ValidityChecker_ptr = ValidityCheckerTpl_ptr<DATATYPE>;


    DEFINE_TEMPLATE_EIGEN(DATATYPE)

    PlanningWorldTpl_ptr<DATATYPE> world;
    
    typedef struct PlanningConfig {
        CompoundStateSpace_ptr cs;
        SpaceInformation_ptr si;
        ProblemDefinition_ptr pdef;
        ValidityCheckerTpl_ptr<DATATYPE> valid_checker;
        std::vector<DATATYPE> lower_joint_limits, upper_joint_limits;
        std::vector<bool> is_revolute;
        FixedJoints fixed_joints;
        size_t dim;
    } PlanningConfig;

    std::map<FixedJoints, PlanningConfig> planning_configs;

public:
    VectorX random_sample_nearby(VectorX const &start_state, PlanningConfig &planning_config);

    OMPLPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world);

    /**
     * @brief build a new state space given the current planning world
     *        and a set of fixed joints
     * 
     * @param fixed_joints a vector of FixedJoint
     */
    void build_planning_config(FixedJoints const &fixed_joints = FixedJoints());

    PlanningWorldTpl_ptr<DATATYPE> get_world() { return world; }

    std::pair <std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
    plan(const VectorX &start_state,
         const std::vector<VectorX> &goal_states,
         const std::string &planner_name = "RRTConnect",
         const double &time = 1.0,
         const double& range = 0.0,
         const bool &verbose = false,
         const FixedJoints &fixed_joints = FixedJoints());
};


template<typename DATATYPE>
using OMPLPlannerTpl_ptr = std::shared_ptr<ValidityCheckerTpl<DATATYPE>>;


using OMPLPlannerTpld_ptr = OMPLPlannerTpl_ptr<double>;
using OMPLPlannerTplf_ptr = OMPLPlannerTpl_ptr<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplf = OMPLPlannerTpl<float>;

