#pragma once

#include <memory>

#include <hpp/fcl/broadphase/broadphase_collision_manager.h>
// #include <hpp/fcl/geometry/bvh/BVH_model.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/convex.h>
#include <hpp/fcl/collision.h>

namespace mplib::collision_detection::fcl {

// Namespace alias
namespace fcl = ::hpp::fcl;

}  // namespace mplib::collision_detection::fcl

namespace hpp::fcl {

using CollisionGeometryPtr = std::shared_ptr<CollisionGeometry>;

using ConvexPtr = std::shared_ptr<Convex<Triangle>>;

using BVHModel_OBBRSS = BVHModel<OBBRSS>;

using BVHModel_OBBRSSPtr = std::shared_ptr<BVHModel_OBBRSS>;

using CollisionObjectPtr = std::shared_ptr<CollisionObject>;

using BroadPhaseCollisionManagerPtr =
    std::shared_ptr<BroadPhaseCollisionManager>;

}  // namespace fcl
