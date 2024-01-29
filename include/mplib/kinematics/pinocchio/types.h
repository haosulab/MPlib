#pragma once

#include <string_view>

#include <pinocchio/parsers/urdf.hpp>

namespace mplib::kinematics::pinocchio {

// Namespace alias
namespace pinocchio = ::pinocchio;

}  // namespace mplib::kinematics::pinocchio

namespace pinocchio {

template <typename S>
using UrdfVisitorBase = pinocchio::urdf::details::UrdfVisitorBaseTpl<S, 0>;

// Pinocchio joint_type prefix (i.e., JointModel shortname prefix)
// https://github.com/stack-of-tasks/pinocchio/blob/97a00a983e66fc0a58667f9671e2d806ba9b730b/include/pinocchio/bindings/python/multibody/joint/joint.hpp#L48-L58
inline constexpr std::string_view joint_type_prefix {"JointModel"};

}  // namespace pinocchio
