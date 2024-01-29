#pragma once

#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>

#include "mplib/types.h"

namespace mplib {

template <typename S>
Isometry3<S> toIsometry(const pinocchio::SE3Tpl<S> &T);

template <typename S>
Isometry3<S> toIsometry(const urdf::Pose &M);

template <typename S>
pinocchio::SE3Tpl<S> toSE3(const Isometry3<S> &T);

template <typename S>
pinocchio::SE3Tpl<S> toSE3(const urdf::Pose &M);

template <typename S>
pinocchio::InertiaTpl<S> convertInertial(const urdf::Inertial &Y);

template <typename S>
pinocchio::InertiaTpl<S> convertInertial(const urdf::InertialSharedPtr &Y);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_CONVERSION(S)                                       \
  extern template Isometry3<S> toIsometry<S>(const pinocchio::SE3Tpl<S> &T); \
  extern template Isometry3<S> toIsometry<S>(const urdf::Pose &M);           \
  extern template pinocchio::SE3Tpl<S> toSE3<S>(const Isometry3<S> &T);      \
  extern template pinocchio::SE3Tpl<S> toSE3<S>(const urdf::Pose &M);        \
  extern template pinocchio::InertiaTpl<S> convertInertial<S>(               \
      const urdf::Inertial &Y);                                              \
  extern template pinocchio::InertiaTpl<S> convertInertial<S>(               \
      const urdf::InertialSharedPtr &Y)

DECLARE_TEMPLATE_CONVERSION(float);
DECLARE_TEMPLATE_CONVERSION(double);

}  // namespace mplib
