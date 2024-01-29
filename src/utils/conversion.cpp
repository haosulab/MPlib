#include "mplib/utils/conversion.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_CONVERSION(S)                                            \
  template Isometry3<S> toIsometry<S>(const pinocchio::SE3Tpl<S> &T);            \
  template Isometry3<S> toIsometry<S>(const urdf::Pose &M);                      \
  template pinocchio::SE3Tpl<S> toSE3<S>(const Isometry3<S> &T);                 \
  template pinocchio::SE3Tpl<S> toSE3<S>(const urdf::Pose &M);                   \
  template pinocchio::InertiaTpl<S> convertInertial<S>(const urdf::Inertial &Y); \
  template pinocchio::InertiaTpl<S> convertInertial<S>(const urdf::InertialSharedPtr &Y)

DEFINE_TEMPLATE_CONVERSION(float);
DEFINE_TEMPLATE_CONVERSION(double);

template <typename S>
Isometry3<S> toIsometry(const pinocchio::SE3Tpl<S> &T) {
  Isometry3<S> ret;
  ret.linear() = T.rotation_impl();
  ret.translation() = T.translation_impl();
  return ret;
}

template <typename S>
Isometry3<S> toIsometry(const urdf::Pose &M) {
  const urdf::Vector3 &p = M.position;
  const urdf::Rotation &q = M.rotation;
  Isometry3<S> ret;
  ret.linear() = Quaternion<S> {static_cast<S>(q.w), static_cast<S>(q.x),
                                static_cast<S>(q.y), static_cast<S>(q.z)}
                     .matrix();
  ret.translation() =
      Vector3<S> {static_cast<S>(p.x), static_cast<S>(p.y), static_cast<S>(p.z)};
  return ret;
}

template <typename S>
pinocchio::SE3Tpl<S> toSE3(const Isometry3<S> &T) {
  return pinocchio::SE3Tpl<S>(T.linear(), T.translation());
}

template <typename S>
pinocchio::SE3Tpl<S> toSE3(const urdf::Pose &M) {
  const urdf::Vector3 &p = M.position;
  const urdf::Rotation &q = M.rotation;
  return pinocchio::SE3Tpl<S>(
      Quaternion<S> {static_cast<S>(q.w), static_cast<S>(q.x), static_cast<S>(q.y),
                     static_cast<S>(q.z)}
          .matrix(),
      Vector3<S> {static_cast<S>(p.x), static_cast<S>(p.y), static_cast<S>(p.z)});
}

template <typename S>
pinocchio::InertiaTpl<S> convertInertial(const urdf::Inertial &Y) {
  const urdf::Vector3 &p = Y.origin.position;
  const urdf::Rotation &q = Y.origin.rotation;
  const Vector3<S> com {static_cast<S>(p.x), static_cast<S>(p.y), static_cast<S>(p.z)};
  const Matrix3<S> &R = Quaternion<S> {static_cast<S>(q.w), static_cast<S>(q.x),
                                       static_cast<S>(q.y), static_cast<S>(q.z)}
                            .matrix();
  Matrix3<S> I;
  I << Y.ixx, Y.ixy, Y.ixz, Y.ixy, Y.iyy, Y.iyz, Y.ixz, Y.iyz, Y.izz;
  return pinocchio::InertiaTpl<S>(Y.mass, com, R * I * R.transpose());
}

template <typename S>
pinocchio::InertiaTpl<S> convertInertial(const urdf::InertialSharedPtr &Y) {
  if (Y) return convertInertial<S>(*Y);
  return pinocchio::InertiaTpl<S>::Zero();
}

}  // namespace mplib
