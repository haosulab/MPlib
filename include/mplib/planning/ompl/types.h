#pragma once

#include <memory>

#include <ompl/base/State.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

namespace mplib::planning::ompl {

// Namespace alias
namespace ob = ::ompl::base;
// namespace oc = ::ompl::control;
namespace og = ::ompl::geometric;

}  // namespace mplib::planning::ompl

namespace ompl::base {

using CompoundStateSpacePtr = std::shared_ptr<CompoundStateSpace>;
using RealVectorStateSpacePtr = std::shared_ptr<RealVectorStateSpace>;
using GoalStatesPtr = std::shared_ptr<GoalStates>;

}  // namespace ompl::base
