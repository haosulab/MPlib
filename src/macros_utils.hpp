#pragma once
#ifndef USE_TEMPLATE_EIGEN
#define USE_TEMPLATE_EIGEN
#define DEFINE_TEMPLATE_EIGEN(DATATYPE) \
    using VectorX = Eigen::Matrix<DATATYPE, Eigen::Dynamic, 1>;\
    using Vector3 = Eigen::Matrix<DATATYPE, 3, 1>;\
    using Vector4 = Eigen::Matrix<DATATYPE, 4, 1>;\
    using Vector6 = Eigen::Matrix<DATATYPE, 6, 1>;\
    using Vector7 = Eigen::Matrix<DATATYPE, 7, 1>;\
    using Vector3I = Eigen::Matrix<int, 3, 1>;\
    using Quaternion = Eigen::Quaternion<DATATYPE>;\
    using Matrix3 = Eigen::Matrix<DATATYPE, 3, 3>;\
    using Matrix6 = Eigen::Matrix<DATATYPE, 6, 6>;\
    using Matrix6x = Eigen::Matrix<DATATYPE, 6, Eigen::Dynamic>;\
    using Matrixx3 = Eigen::Matrix<DATATYPE, Eigen::Dynamic, 3>;\
    using Matrixx3I = Eigen::Matrix<int, Eigen::Dynamic, 3>;\
    using VectorXI = Eigen::VectorXi;\
    using Transform3 = Eigen::Transform<DATATYPE, 3, Eigen::Isometry>;
#endif


#ifndef USE_TEMPLATE_PINOCCHIO
#define USE_TEMPLATE_PINOCCHIO
#define DEFINE_TEMPLATE_PINOCCHIO(DATATYPE) \
    using Model = pinocchio::ModelTpl<DATATYPE>;\
    using Data = pinocchio::DataTpl<DATATYPE>;\
    using UrdfVisitorBase = pinocchio::urdf::details::UrdfVisitorBaseTpl<DATATYPE, 0>;\
    using SE3 = pinocchio::SE3Tpl<DATATYPE>;\
    using Inertia = pinocchio::InertiaTpl<DATATYPE>;\
    using FrameIndex = pinocchio::FrameIndex;
#endif


#ifndef USE_TEMPLATE_FCL
#define USE_TEMPLATE_FCL
#define DEFINE_TEMPLATE_FCL(DATATYPE) \
    using OBBRSS = fcl::OBBRSS<DATATYPE>;\
    using CollisionGeometry = fcl::CollisionGeometry<DATATYPE>;\
    using BVHModel_OBBRSS = fcl::BVHModel<OBBRSS>;\
    using Convex = fcl::Convex<DATATYPE>;\
    using Capsule = fcl::Capsule<DATATYPE>;\
    using Cylinder = fcl::Cylinder<DATATYPE>;                  \
    using Box = fcl::Box<DATATYPE>;\
    using Sphere = fcl::Sphere<DATATYPE>;\
    using Cone = fcl::Cone<DATATYPE>;\
    using Plane = fcl::Plane<DATATYPE>;\
    using CollisionObject = fcl::CollisionObject<DATATYPE>;\
    using CollisionRequest = fcl::CollisionRequest<DATATYPE>;\
    using CollisionResult = fcl::CollisionResult<DATATYPE>;    \
    using DistanceRequest = fcl::DistanceRequest<DATATYPE>;\
    using DistanceResult = fcl::DistanceResult<DATATYPE>;    \
    using Triangle = fcl::Triangle;\
    using ContactPoint = fcl::ContactPoint<DATATYPE>;\
    using Contact = fcl::Contact<DATATYPE>;\
    using CostSource = fcl::CostSource<DATATYPE>;\
    using GJKSolverType = fcl::GJKSolverType;\
    using DynamicAABBTreeCollisionManager = fcl::DynamicAABBTreeCollisionManager<DATATYPE>;\
    using BroadPhaseCollisionManager_ptr = std::shared_ptr<fcl::BroadPhaseCollisionManager<DATATYPE>>;\
    using CollisionGeometry_ptr = std::shared_ptr<CollisionGeometry>;\
    using CollisionObject_ptr = std::shared_ptr<CollisionObject>; \
    using OcTree = fcl::OcTree<DATATYPE>;
#endif


#ifndef USE_ASSERT
#define USE_ASSERT
#define ASSERT(exp, info) if (!(exp)) { throw std::runtime_error((info)); }
#endif

