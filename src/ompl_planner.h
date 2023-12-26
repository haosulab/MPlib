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
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
//namespace oc = ompl::control;
namespace og = ompl::geometric;


template<typename DATATYPE>
std::vector <DATATYPE> compoundstate2vector(const ob::State *state_raw, ob::SpaceInformation *const &si_) {
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

template<typename DATATYPE>
std::vector<DATATYPE> rvssstate2vector(const ob::State *state_raw, ob::SpaceInformation *const &si_) {
    auto dim = si_->getStateDimension();
    auto state = state_raw->as<ob::ProjectedStateSpace::StateType>();
    std::vector<DATATYPE> ret;
    for (size_t i = 0; i < dim; i++) {
        ret.push_back((DATATYPE)(*state)[i]);
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
Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1> state2eigen(const ob::State *state_raw,
                                                       ob::SpaceInformation *const &si_,
                                                       bool is_rvss=false) {
    std::vector<DATATYPE> vec_ret = is_rvss ? rvssstate2vector<DATATYPE>(state_raw, si_)
                                            : compoundstate2vector<DATATYPE>(state_raw, si_);
    auto ret = vector2eigen<DATATYPE, DATATYPE>(vec_ret);
    return ret;
}


template<typename DATATYPE>
class ValidityCheckerTpl : public ob::StateValidityChecker {
    typedef Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1> VectorX;
    PlanningWorldTpl_ptr<DATATYPE> world;
    bool is_rvss;

public:
    ValidityCheckerTpl(PlanningWorldTpl_ptr<DATATYPE> world, const ob::SpaceInformationPtr &si, bool is_rvss)
        : ob::StateValidityChecker(si), world(world), is_rvss(is_rvss) {}
    bool _isValid(VectorX state) const {
        world->setQposAll(state);
        return !world->collide();
    }

    bool isValid(const ob::State *state_raw) const {
        auto state = state2eigen<DATATYPE>(state_raw, si_, is_rvss);
        return _isValid(state);
    }
};

class GeneralConstraint : public ob::Constraint {
    std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> f, j;
public:
    GeneralConstraint(size_t dim, std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> &f,
                      std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> &j)
        : ob::Constraint(dim, 1),
          f(f),
          j(j) {}

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {
        f(x, out);
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override {
        j(x, out);
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

    std::shared_ptr<ob::RealVectorStateSpace> p_ambient_space;
    std::shared_ptr<ob::ProjectedStateSpace> p_constrained_space;
    CompoundStateSpace_ptr cs;
    ob::StateSpacePtr state_space;
    std::shared_ptr<ompl::geometric::SimpleSetup> ss;
    SpaceInformation_ptr p_compound_si;
    SpaceInformation_ptr p_constrained_si;
    SpaceInformation_ptr si;
    PlanningWorldTpl_ptr<DATATYPE> world;
    ValidityCheckerTpl_ptr<DATATYPE> valid_checker;
    size_t dim;
    std::vector<DATATYPE> lower_joint_limits, upper_joint_limits;
    std::vector<bool> is_revolute;
    
    /** Certain joint configurations are the same if some joints are the same fmod 2pi. */
    std::shared_ptr<ob::GoalStates> make_goal_states(std::vector<VectorX> const &goal_states);
    
    /** build a real vector state space for the robot */
    void build_constrained_ambient_state_space();

    /**
     * ss depends on si so every time we change that need to redo it.
     * also need to store the space type since ompl loses it -_-
     */
    void update_ss(bool is_rvss=false);

    void _simplify_path(og::PathGeometric &path);  // keep this private to avoid confusion

public:
    // OMPLPlannerTpl(PlanningWorldTpl_ptr<DATATYPE> const &world);
    
    OMPLPlannerTpl(const PlanningWorldTpl_ptr<DATATYPE> &world, int robot_idx=0);

    VectorX random_sample_nearby(VectorX const &start_state);

    void build_state_space();

    PlanningWorldTpl_ptr<DATATYPE> get_world() { return world; }

    size_t get_dim() { return dim; }

    Eigen::MatrixXd simplify_path(Eigen::MatrixXd &path);

    std::pair <std::string, Eigen::Matrix<DATATYPE, Eigen::Dynamic, Eigen::Dynamic>>
    plan(VectorX const &start_state,
         std::vector<VectorX> const &goal_states,
         const std::string &planner_name = "RRTConnect",
         const double &time = 1.0,
         const double &range = 0.0,
         const bool verbose = false,
         const bool no_simplification = false,
         std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> &constraint_function=nullptr,
         std::function<void(const Eigen::VectorXd &, Eigen::Ref<Eigen::VectorXd>)> &constraint_jacobian=nullptr,
         double constraint_tolerance=1e-3);
};


template<typename DATATYPE>
using OMPLPlannerTpl_ptr = std::shared_ptr<ValidityCheckerTpl<DATATYPE>>;


using OMPLPlannerTpld_ptr = OMPLPlannerTpl_ptr<double>;
using OMPLPlannerTplf_ptr = OMPLPlannerTpl_ptr<float>;
using OMPLPlannerTpld = OMPLPlannerTpl<double>;
using OMPLPlannerTplf = OMPLPlannerTpl<float>;

