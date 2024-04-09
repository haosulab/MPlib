#pragma once

#include "mplib/types.h"

namespace mplib {

/// @brief Pose stored as a unit quaternion and a position vector
template <typename S>
struct Pose {
  /// @brief Constructs a default Pose with p = (0,0,0) and q = (1,0,0,0)
  Pose() : p(0.0, 0.0, 0.0), q(1.0, 0.0, 0.0, 0.0) {}

  /**
   * Constructs a Pose with given position and quaternion
   *
   * @param p: position, format: (x, y, z)
   * @param q: quaternion (can be unnormalized), format: (w, x, y, z)
   */
  Pose(const Vector3<S> &p, const Vector4<S> &q)
      : p(p), q(Quaternion<S> {q(0), q(1), q(2), q(3)}.normalized()) {}

  /**
   * Constructs a Pose with given position and quaternion
   *
   * @param p: position, format: (x, y, z)
   * @param q: quaternion (can be unnormalized), format: (w, x, y, z)
   */
  Pose(const Vector3<S> &p, const Quaternion<S> &q) : p(p), q(q.normalized()) {}

  /**
   * Get the inserse Pose
   *
   * @return: the inverse Pose
   */
  Pose<S> inverse() const {
    Quaternion<S> qinv = q.conjugate();  // can use conjugate() since always normalized
    return {qinv * -p, qinv};
  }

  /// @brief Overloading operator * for ``Pose<S> * Vector3<S>``
  Vector3<S> operator*(const Vector3<S> &v) const { return q * v + p; };

  /// @brief Overloading operator * for ``Pose<S> * Pose<S>``
  Pose<S> operator*(const Pose<S> &other) const {
    return {q * other.p + p, q * other.q};
  };

  /// @brief Overloading operator *= for ``Pose<S> *= Pose<S>``
  Pose<S> &operator*=(const Pose<S> &other) {
    *this = *this * other;
    return *this;
  };

  /// @brief Overloading operator <<
  friend std::ostream &operator<<(std::ostream &out, const Pose<S> &pose) {
    out << "Pose([" << pose.p(0) << ", " << pose.p(1) << ", " << pose.p(2) << "], ["
        << pose.q.w() << ", " << pose.q.x() << ", " << pose.q.y() << ", " << pose.q.z()
        << "])";
    return out;
  }

  Vector3<S> p {0.0, 0.0, 0.0};          ///< Position part of the Pose (x, y, z)
  Quaternion<S> q {1.0, 0.0, 0.0, 0.0};  ///< Quaternion part of the Pose (w, x, y, z)
};

}  // namespace mplib
