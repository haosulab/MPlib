#pragma once

#include <cstdlib>

#include <fcl/math/rng.h>
#include <ompl/util/RandomNumbers.h>

namespace mplib {

/**
 * Sets the global seed for MPlib (``std::srand()``, OMPL's RNG, and FCL's RNG).
 *
 * @param seed: the random seed value
 */
template <typename S>
inline void setGlobalSeed(unsigned seed) {
  std::srand(seed);
  ::ompl::RNG::setSeed(seed);
  ::fcl::RNG<S>::setSeed(seed);
}

}  // namespace mplib
