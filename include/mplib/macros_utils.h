#pragma once
#ifndef USE_TEMPLATE_EIGEN
#define USE_TEMPLATE_EIGEN
#define DEFINE_TEMPLATE_EIGEN(S)                           \
  using VectorX = Eigen::Matrix<S, Eigen::Dynamic, 1>;     \
  using Vector3 = Eigen::Matrix<S, 3, 1>;                  \
  using Vector4 = Eigen::Matrix<S, 4, 1>;                  \
  using Vector6 = Eigen::Matrix<S, 6, 1>;                  \
  using Vector7 = Eigen::Matrix<S, 7, 1>;                  \
  using Vector3I = Eigen::Matrix<int, 3, 1>;               \
  using Quaternion = Eigen::Quaternion<S>;                 \
  using Matrix3 = Eigen::Matrix<S, 3, 3>;                  \
  using Matrix6 = Eigen::Matrix<S, 6, 6>;                  \
  using Matrix6x = Eigen::Matrix<S, 6, Eigen::Dynamic>;    \
  using Matrixx3 = Eigen::Matrix<S, Eigen::Dynamic, 3>;    \
  using Matrixx3I = Eigen::Matrix<int, Eigen::Dynamic, 3>; \
  using VectorXI = Eigen::VectorXi;                        \
  using Transform3 = Eigen::Transform<S, 3, Eigen::Isometry>;
#endif

#ifndef USE_TEMPLATE_PINOCCHIO
#define USE_TEMPLATE_PINOCCHIO
#define DEFINE_TEMPLATE_PINOCCHIO(S)                                          \
  using Model = pinocchio::ModelTpl<S>;                                       \
  using Data = pinocchio::DataTpl<S>;                                         \
  using UrdfVisitorBase = pinocchio::urdf::details::UrdfVisitorBaseTpl<S, 0>; \
  using SE3 = pinocchio::SE3Tpl<S>;                                           \
  using Inertia = pinocchio::InertiaTpl<S>;                                   \
  using FrameIndex = pinocchio::FrameIndex;
#endif

#ifndef USE_TEMPLATE_FCL
#define USE_TEMPLATE_FCL
#define DEFINE_TEMPLATE_FCL(S)                                                     \
  using OBBRSS = fcl::OBBRSS<S>;                                                   \
  using CollisionGeometry = fcl::CollisionGeometry<S>;                             \
  using BVHModel_OBBRSS = fcl::BVHModel<OBBRSS>;                                   \
  using Convex = fcl::Convex<S>;                                                   \
  using Capsule = fcl::Capsule<S>;                                                 \
  using Cylinder = fcl::Cylinder<S>;                                               \
  using Box = fcl::Box<S>;                                                         \
  using Sphere = fcl::Sphere<S>;                                                   \
  using Cone = fcl::Cone<S>;                                                       \
  using Plane = fcl::Plane<S>;                                                     \
  using CollisionObject = fcl::CollisionObject<S>;                                 \
  using CollisionRequest = fcl::CollisionRequest<S>;                               \
  using CollisionResult = fcl::CollisionResult<S>;                                 \
  using DistanceRequest = fcl::DistanceRequest<S>;                                 \
  using DistanceResult = fcl::DistanceResult<S>;                                   \
  using Triangle = fcl::Triangle;                                                  \
  using ContactPoint = fcl::ContactPoint<S>;                                       \
  using Contact = fcl::Contact<S>;                                                 \
  using CostSource = fcl::CostSource<S>;                                           \
  using GJKSolverType = fcl::GJKSolverType;                                        \
  using DynamicAABBTreeCollisionManager = fcl::DynamicAABBTreeCollisionManager<S>; \
  using BroadPhaseCollisionManagerPtr =                                            \
      std::shared_ptr<fcl::BroadPhaseCollisionManager<S>>;                         \
  using CollisionGeometryPtr = std::shared_ptr<CollisionGeometry>;                 \
  using CollisionObjectPtr = std::shared_ptr<CollisionObject>;                     \
  using OcTree = fcl::OcTree<S>;
#endif

#ifndef USE_ASSERT
#define USE_ASSERT
#define ASSERT(exp, info)             \
  if (!(exp)) {                       \
    throw std::runtime_error((info)); \
  }
#endif
