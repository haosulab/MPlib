#pragma once

#include <Eigen/Dense>

namespace mplib {

// Eigen ===============================================================================
template <typename S>
using Vector3 = Eigen::Matrix<S, 3, 1>;

template <typename S>
using Vector4 = Eigen::Matrix<S, 4, 1>;

template <typename S>
using Vector6 = Eigen::Matrix<S, 6, 1>;

template <typename S>
using VectorX = Eigen::Matrix<S, Eigen::Dynamic, 1>;

using Vector3i = Eigen::Vector3i;
using VectorXi = Eigen::VectorXi;
using VectorXd = Eigen::VectorXd;

template <typename S>
using Matrix3 = Eigen::Matrix<S, 3, 3>;

template <typename S>
using Matrix6 = Eigen::Matrix<S, 6, 6>;

template <typename S>
using Matrix6X = Eigen::Matrix<S, 6, Eigen::Dynamic>;

template <typename S>
using MatrixX3 = Eigen::Matrix<S, Eigen::Dynamic, 3>;

template <typename S>
using MatrixX = Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>;

using MatrixX3i = Eigen::Matrix<int, Eigen::Dynamic, 3>;
using MatrixXd = Eigen::MatrixXd;

using PermutationMatrixX = Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>;

template <typename S>
using Quaternion = Eigen::Quaternion<S>;

template <typename S>
using Isometry3 = Eigen::Transform<S, 3, Eigen::Isometry>;

}  // namespace mplib
