#pragma once

#include "mplib/types.h"

namespace mplib {

/**
 * Pose stored as a unit quaternion and a position vector
 *
 * This struct is intended to be used only for interfacing with Python.
 * Internally, ``Pose`` is converted to and stored as ``Eigen::Isometry3``
 * which is used by all computations.
 */
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
   * Constructs a Pose from a given ``Eigen::Isometry3`` instance
   *
   * @param pose: an ``Eigen::Isometry3`` instance
   */
  Pose(const Isometry3<S> &pose) : p(pose.translation()), q(pose.linear()) {}

  /**
   * Converts the Pose to an ``Eigen::Isometry3`` instance
   *
   * @return: an ``Eigen::Isometry3`` instance
   */
  Isometry3<S> toIsometry() const {
    Isometry3<S> ret;
    ret.linear() = q.toRotationMatrix();
    ret.translation() = p;
    return ret;
  }

  /**
   * Get the inserse Pose
   *
   * @return: the inverse Pose
   */
  Pose<S> inverse() const {
    Quaternion<S> qinv = q.conjugate();  // can use conjugate() since always normalized
    return {qinv * -p, qinv};
  }

  /**
   * Computes the distance between two poses by
   * ``norm(p1.p - p2.p) + min(norm(p1.q - p2.q), norm(p1.q + p2.q))`.
   *
   * The quaternion part has range [0, sqrt(2)].
   *
   * @param other: the other pose
   * @return: the distance between the two poses
   */
  S distance(const Pose<S> &other) const {
    return (p - other.p).norm() + std::min((q.coeffs() - other.q.coeffs()).norm(),
                                           (q.coeffs() + other.q.coeffs()).norm());
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
