#pragma once

#include <memory>
#include <set>

#include <Eigen/Dense>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/fwd.hpp>

namespace mplib {

// Eigen
template <typename S>
using Vector3 = Eigen::Matrix<S, 3, 1>;

template <typename S>
using Vector4 = Eigen::Matrix<S, 4, 1>;

template <typename S>
using Vector6 = Eigen::Matrix<S, 6, 1>;

template <typename S>
using Vector7 = Eigen::Matrix<S, 7, 1>;

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
using Transform3 = Eigen::Transform<S, 3, Eigen::Isometry>;

// Pinocchio ===========================================================================
namespace pinocchio {

template <typename S>
using Model = ::pinocchio::ModelTpl<S>;

template <typename S>
using Data = ::pinocchio::DataTpl<S>;

using FrameIndex = ::pinocchio::FrameIndex;

template <typename S>
using UrdfVisitorBase = ::pinocchio::urdf::details::UrdfVisitorBaseTpl<S, 0>;

template <typename S>
using SE3 = ::pinocchio::SE3Tpl<S>;

template <typename S>
using Inertia = ::pinocchio::InertiaTpl<S>;

// Forward declaration so that we can export some classes to mplib namespace
template <typename S>
class PinocchioModelTpl;

}  // namespace pinocchio

// FCL =================================================================================
namespace fcl {

using Triangle = ::fcl::Triangle;

template <typename S>
using CollisionGeometry = ::fcl::CollisionGeometry<S>;

template <typename S>
using CollisionGeometryPtr = std::shared_ptr<CollisionGeometry<S>>;

template <typename S>
using Box = ::fcl::Box<S>;

template <typename S>
using Capsule = ::fcl::Capsule<S>;

template <typename S>
using Cone = ::fcl::Cone<S>;

template <typename S>
using Convex = ::fcl::Convex<S>;

template <typename S>
using Cylinder = ::fcl::Cylinder<S>;

template <typename S>
using Plane = ::fcl::Plane<S>;

template <typename S>
using Sphere = ::fcl::Sphere<S>;

template <typename S>
using OBBRSS = ::fcl::OBBRSS<S>;

template <typename BV>
using BVHModel = ::fcl::BVHModel<BV>;

template <typename S>
using BVHModel_OBBRSS = BVHModel<OBBRSS<S>>;

template <typename S>
using OcTree = ::fcl::OcTree<S>;

template <typename S>
using CollisionObject = ::fcl::CollisionObject<S>;

template <typename S>
using CollisionObjectPtr = std::shared_ptr<CollisionObject<S>>;

using GJKSolverType = ::fcl::GJKSolverType;

template <typename S>
using CollisionRequest = ::fcl::CollisionRequest<S>;

template <typename S>
using CollisionResult = ::fcl::CollisionResult<S>;

template <typename S>
using DistanceRequest = ::fcl::DistanceRequest<S>;

template <typename S>
using DistanceResult = ::fcl::DistanceResult<S>;

template <typename S>
using Contact = ::fcl::Contact<S>;

template <typename S>
using ContactPoint = ::fcl::ContactPoint<S>;

template <typename S>
using CostSource = ::fcl::CostSource<S>;

template <typename S>
using DynamicAABBTreeCollisionManager = ::fcl::DynamicAABBTreeCollisionManager<S>;

template <typename S>
using BroadPhaseCollisionManagerPtr =
    std::shared_ptr<::fcl::BroadPhaseCollisionManager<S>>;

// Forward declaration so that we can export some classes to mplib namespace
template <typename S>
class FCLModelTpl;

}  // namespace fcl

// KDL =================================================================================
namespace kdl {

// Forward declaration so that we can export some classes to mplib namespace
template <typename S>
class KDLModelTpl;

}  // namespace kdl

// OMPL ================================================================================
namespace ompl {

// namespace alias
namespace ob = ::ompl::base;
// namespace oc = ::ompl::control;
namespace og = ::ompl::geometric;

// common type alias
using CompoundStateSpace = ob::CompoundStateSpace;
using CompoundStateSpacePtr = std::shared_ptr<ob::CompoundStateSpace>;
using SpaceInformation = ob::SpaceInformation;
using SpaceInformationPtr = std::shared_ptr<ob::SpaceInformation>;
using ProblemDefinition = ob::ProblemDefinition;
using ProblemDefinitionPtr = std::shared_ptr<ob::ProblemDefinition>;

// FixedJoint / FixedJoints
struct FixedJoint;
using FixedJoints = std::set<FixedJoint>;

}  // namespace ompl

// Export some classes defined in inner namespace to mplib namespace
template <typename S>
using PinocchioModelTpl = pinocchio::PinocchioModelTpl<S>;

template <typename S>
using FCLModelTpl = fcl::FCLModelTpl<S>;

template <typename S>
using KDLModelTpl = kdl::KDLModelTpl<S>;

}  // namespace mplib
