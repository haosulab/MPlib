#pragma once

#include <vector>

#include "mplib/planning/ompl/types.h"
#include "mplib/types.h"

namespace mplib::planning::ompl {

template <typename S>
std::vector<S> compoundState2Vector(const ob::State *state_raw,
                                    const ob::SpaceInformation *si);

template <typename S>
std::vector<S> rvssState2Vector(const ob::State *state_raw,
                                const ob::SpaceInformation *si);

template <typename IN_TYPE, typename OUT_TYPE>
std::vector<OUT_TYPE> eigen2Vector(const VectorX<IN_TYPE> &v) {
  std::vector<OUT_TYPE> ret;
  for (const auto &x : v) ret.push_back(static_cast<OUT_TYPE>(x));
  return ret;
}

template <typename IN_TYPE, typename OUT_TYPE>
VectorX<OUT_TYPE> vector2Eigen(const std::vector<IN_TYPE> &v) {
  VectorX<OUT_TYPE> ret(v.size());
  for (size_t i = 0; i < v.size(); i++) ret[i] = static_cast<OUT_TYPE>(v[i]);
  return ret;
}

/**
 * Convert a ompl::base::State to Eigen::VectorX.
 *
 * @param state_raw: pointer to a raw state.
 * @param si: pointer to ompl::base::SpaceInformation.
 * @param is_rvss: whether the state space is an ompl::base::RealVectorStateSpace.
 *    If ``true``, we are using constrained planning.
 * @return: an Eigen::VectorX of the ompl::base::State.
 */
template <typename S>
VectorX<S> state2Eigen(const ob::State *state_raw, const ob::SpaceInformation *si,
                       bool is_rvss = false);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_OMPL_UTILS(S)                                                \
  extern template std::vector<S> compoundState2Vector<S>(                             \
      const ob::State *state_raw, const ob::SpaceInformation *si);                    \
  extern template std::vector<S> rvssState2Vector<S>(const ob::State *state_raw,      \
                                                     const ob::SpaceInformation *si); \
  extern template VectorX<S> state2Eigen<S>(                                          \
      const ob::State *state_raw, const ob::SpaceInformation *si, bool is_rvss)

DECLARE_TEMPLATE_OMPL_UTILS(float);
DECLARE_TEMPLATE_OMPL_UTILS(double);

}  // namespace mplib::planning::ompl
