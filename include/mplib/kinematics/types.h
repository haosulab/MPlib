#pragma once

#include "mplib/kinematics/kdl/kdl_model.h"
#include "mplib/kinematics/pinocchio/pinocchio_model.h"

namespace mplib::kinematics {

// Export classes from inner namespace to mplib::kinematics namespace
template <typename S>
using KDLModelTpl = kdl::KDLModelTpl<S>;

template <typename S>
using KDLModelTplPtr = kdl::KDLModelTplPtr<S>;

template <typename S>
using PinocchioModelTpl = pinocchio::PinocchioModelTpl<S>;

template <typename S>
using PinocchioModelTplPtr = pinocchio::PinocchioModelTplPtr<S>;

}  // namespace mplib::kinematics
