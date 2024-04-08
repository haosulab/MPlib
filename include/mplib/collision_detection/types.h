#pragma once

#include "mplib/collision_detection/collision_common.h"
#include "mplib/collision_detection/fcl/collision_common.h"
#include "mplib/collision_detection/fcl/fcl_model.h"

namespace mplib {

namespace collision_detection {

// Export classes from inner namespace to mplib::collision_detection namespace
template <typename S>
using FCLModelTpl = fcl::FCLModelTpl<S>;

template <typename S>
using FCLModelTplPtr = fcl::FCLModelTplPtr<S>;

template <typename S>
using FCLObject = fcl::FCLObject<S>;

template <typename S>
using FCLObjectPtr = fcl::FCLObjectPtr<S>;

}  // namespace collision_detection

// Export classes from inner namespace to mplib namespace
template <typename S>
using WorldCollisionResultTpl = collision_detection::WorldCollisionResultTpl<S>;

template <typename S>
using WorldDistanceResultTpl = collision_detection::WorldDistanceResultTpl<S>;

}  // namespace mplib
