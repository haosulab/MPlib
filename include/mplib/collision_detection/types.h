#pragma once

#include "mplib/collision_detection/collision_common.h"
#include "mplib/collision_detection/fcl/fcl_model.h"

namespace mplib {

namespace collision_detection {

// Export classes from inner namespace to mplib::collision_detection namespace
template <typename S>
using FCLModelTpl = fcl::FCLModelTpl<S>;

template <typename S>
using FCLModelTplPtr = fcl::FCLModelTplPtr<S>;

}  // namespace collision_detection

// Export classes from inner namespace to mplib namespace
template <typename S>
using WorldCollisionResultTpl = collision_detection::WorldCollisionResultTpl<S>;

}  // namespace mplib
