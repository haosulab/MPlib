#include "mplib/planning/ompl/ompl_utils.h"

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

namespace mplib::planning::ompl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_OMPL_UTILS(S)                                              \
  template std::vector<S> compoundState2Vector<S>(const ob::State *state_raw,      \
                                                  const ob::SpaceInformation *si); \
  template std::vector<S> rvssState2Vector<S>(const ob::State *state_raw,          \
                                              const ob::SpaceInformation *si);     \
  template VectorX<S> state2Eigen<S>(const ob::State *state_raw,                   \
                                     const ob::SpaceInformation *si, bool is_rvss)

DEFINE_TEMPLATE_OMPL_UTILS(float);
DEFINE_TEMPLATE_OMPL_UTILS(double);

template <typename S>
std::vector<S> compoundState2Vector(const ob::State *state_raw,
                                    const ob::SpaceInformation *si) {
  const auto state = state_raw->as<ob::CompoundState>();
  const auto cs = si->getStateSpace()->as<ob::CompoundStateSpace>();

  std::vector<S> ret;
  for (size_t i = 0; i < cs->getSubspaceCount(); i++) {
    const auto subspace = cs->getSubspace(i);
    switch (subspace->getType()) {
      case ob::STATE_SPACE_REAL_VECTOR:
        for (size_t j = 0; j < subspace->as<ob::RealVectorStateSpace>()->getDimension();
             j++)
          ret.push_back(static_cast<S>(
              (*state)[i]->as<ob::RealVectorStateSpace::StateType>()->values[j]));
        break;
      case ob::STATE_SPACE_SO2:
        ret.push_back(
            static_cast<S>((*state)[i]->as<ob::SO2StateSpace::StateType>()->value));
        break;
      default:
        throw std::invalid_argument("Unhandled subspace type.");
        break;
    }
  }
  return ret;
}

template <typename S>
std::vector<S> rvssState2Vector(const ob::State *state_raw,
                                const ob::SpaceInformation *si) {
  const auto state = state_raw->as<ob::ProjectedStateSpace::StateType>();
  auto dim = si->getStateDimension();
  std::vector<S> ret;
  for (size_t i = 0; i < dim; i++) ret.push_back(static_cast<S>((*state)[i]));
  return ret;
}

template <typename S>
VectorX<S> state2Eigen(const ob::State *state_raw, const ob::SpaceInformation *si,
                       bool is_rvss) {
  std::vector<S> state_vec = is_rvss ? rvssState2Vector<S>(state_raw, si)
                                     : compoundState2Vector<S>(state_raw, si);
  return vector2Eigen<S, S>(state_vec);
}

}  // namespace mplib::planning::ompl
