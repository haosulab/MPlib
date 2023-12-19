#include <pinocchio/fwd.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
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


template<typename DATATYPE>
class ValidityCheckerTpl : public ob::StateValidityChecker {
    typedef Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1> VectorX;
    PlanningWorldTpl_ptr<DATATYPE> world;

public:
    ValidityCheckerTpl(PlanningWorldTpl_ptr<DATATYPE> world, const ob::SpaceInformationPtr &si)
            : ob::StateValidityChecker(si), world(world) {}

    bool isValid(const ob::State *state_raw) const {
        //std::cout << "Begin to check state" << std::endl;
        //std::cout << "check " << state2eigen<DATATYPE>(state_raw, si_) << std::endl;
        world->setQposAll(state2eigen<DATATYPE>(state_raw, si_));
        return !world->collide();
    }

    bool _isValid(VectorX state) const {
        world->setQposAll(state);
        return !world->collide();
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

    CompoundStateSpace_ptr cs;
    SpaceInformation_ptr si;
    ProblemDefinition_ptr pdef;
    PlanningWorldTpl_ptr<DATATYPE> world;
    ValidityCheckerTpl_ptr<DATATYPE> valid_checker;
    size_t dim;
    std::vector<DATATYPE> lower_joint_limits, upper_joint_limits;
    std::vector<bool> is_revolute;

    void _simplify_path(og::PathGeometricPtr path);  // keep this private to avoid confusion

public:
    VectorX random_sample_nearby(VectorX const &start_state);

    OMPLPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world);

    void build_state_space();

    PlanningWorldTpl_ptr<DATATYPE> get_world() { return world; }

    size_t get_dim() { return dim; }

    Eigen::MatrixXd simplify_path(Eigen::MatrixXd &path);

    std::pair <std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
    plan(VectorX const &start_state, std::vector<VectorX> const &goal_states, const std::string &planner_name = "RRTConnect",
        const double &time = 1.0, const double& range = 0.0, const bool &verbose = false);
};


template<typename DATATYPE>
using OMPLPlannerTpl_ptr = std::shared_ptr<ValidityCheckerTpl<DATATYPE>>;


using OMPLPlannerTpld_ptr = OMPLPlannerTpl_ptr<double>;
using OMPLPlannerTplf_ptr = OMPLPlannerTpl_ptr<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplf = OMPLPlannerTpl<float>;

