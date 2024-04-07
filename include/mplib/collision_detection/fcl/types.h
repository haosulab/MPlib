#pragma once

#include <memory>

#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/narrowphase/collision_object.h>

namespace mplib::collision_detection::fcl {

// Namespace alias
namespace fcl = ::fcl;

}  // namespace mplib::collision_detection::fcl

namespace fcl {

template <typename S>
using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometry<S>>;

template <typename S>
using ConvexPtr = std::shared_ptr<fcl::Convex<S>>;

template <typename S>
using BVHModel_OBBRSS = fcl::BVHModel<fcl::OBBRSS<S>>;

template <typename S>
using BVHModel_OBBRSSPtr = std::shared_ptr<BVHModel_OBBRSS<S>>;

template <typename S>
using CollisionObjectPtr = std::shared_ptr<fcl::CollisionObject<S>>;

template <typename S>
using BroadPhaseCollisionManagerPtr =
    std::shared_ptr<fcl::BroadPhaseCollisionManager<S>>;

}  // namespace fcl
