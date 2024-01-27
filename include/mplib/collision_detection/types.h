#pragma once

#include "mplib/collision_detection/fcl/fcl_model.h"

namespace mplib::collision_detection {

// Export classes from inner namespace to mplib::collision_detection namespace
template <typename S>
using FCLModelTpl = fcl::FCLModelTpl<S>;

template <typename S>
using FCLModelTplPtr = fcl::FCLModelTplPtr<S>;

}  // namespace mplib::collision_detection
