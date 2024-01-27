#pragma once

#include <pinocchio/parsers/urdf.hpp>

namespace mplib::kinematics::pinocchio {

// Namespace alias
namespace pinocchio = ::pinocchio;

}  // namespace mplib::kinematics::pinocchio

namespace pinocchio {

template <typename S>
using UrdfVisitorBase = pinocchio::urdf::details::UrdfVisitorBaseTpl<S, 0>;

}  // namespace pinocchio
