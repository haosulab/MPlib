#pragma once

#include "mplib/planning/ompl/ompl_planner.h"

namespace mplib::planning {

// Export classes from inner namespace to mplib::planning namespace
template <typename S>
using OMPLPlannerTpl = ompl::OMPLPlannerTpl<S>;

template <typename S>
using OMPLPlannerTplPtr = ompl::OMPLPlannerTplPtr<S>;

}  // namespace mplib::planning
