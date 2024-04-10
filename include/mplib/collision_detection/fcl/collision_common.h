#pragma once

#include <string>
#include <vector>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

#include "mplib/collision_detection/fcl/types.h"
#include "mplib/macros/class_forward.h"
#include "mplib/types.h"
#include "mplib/utils/pose.h"

namespace mplib::collision_detection::fcl {

// FCLObjectPtr
MPLIB_STRUCT_TEMPLATE_FORWARD(FCLObject);

/**
 * A general high-level object which consists of multiple FCLCollisionObjects.
 * It is the top level data structure which is used in the collision checking process.
 *
 * Mimicking MoveIt2's ``collision_detection::FCLObject`` and
 * ``collision_detection::World::Object``
 *
 * https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1FCLObject.html
 * https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1World_1_1Object.html
 */
template <typename S>
struct FCLObject {
  /**
   * Construct a new FCLObject with the given name
   *
   * @param name: name of this FCLObject
   */
  FCLObject(const std::string &name) : name(name), pose(Isometry3<S>::Identity()) {}

  /**
   * Construct a new FCLObject with the given name and shapes
   *
   * @param name: name of this FCLObject
   * @param pose: pose of this FCLObject. All shapes are relative to this pose
   * @param shapes: all collision shapes as a vector of ``fcl::CollisionObjectPtr``
   * @param shape_poses: relative poses from this FCLObject to each collision shape
   */
  FCLObject(const std::string &name_, const Pose<S> &pose_,
            const std::vector<fcl::CollisionObjectPtr<S>> &shapes_,
            const std::vector<Pose<S>> &shape_poses_);

  std::string name;   ///< Name of this FCLObject
  Isometry3<S> pose;  ///< Pose of this FCLObject. All shapes are relative to this pose
  /// All collision shapes (``fcl::CollisionObjectPtr``) making up this FCLObject
  std::vector<fcl::CollisionObjectPtr<S>> shapes;
  /// Relative poses from this FCLObject to each collision shape
  std::vector<Isometry3<S>> shape_poses;
};

/**
 * Collision function between two ``FCLObject``
 *
 * @param[in] obj1: the first object
 * @param[in] obj2: the second object
 * @param[in] request: ``fcl::CollisionRequest``
 * @param[out] result: ``fcl::CollisionResult``
 * @return: number of contacts generated between the two objects
 */
template <typename S>
size_t collide(const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,
               const fcl::CollisionRequest<S> &request,
               fcl::CollisionResult<S> &result);

/**
 * Distance function between two ``FCLObject``
 *
 * @param[in] obj1: the first object
 * @param[in] obj2: the second object
 * @param[in] request: ``fcl::DistanceRequest``
 * @param[out] result: ``fcl::DistanceResult``
 * @return: minimum distance generated between the two objects
 */
template <typename S>
S distance(const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,
           const fcl::DistanceRequest<S> &request, fcl::DistanceResult<S> &result);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_FCL_COMMON(S)                                           \
  extern template struct FCLObject<S>;                                           \
  extern template size_t collide<S>(                                             \
      const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,                  \
      const fcl::CollisionRequest<S> &request, fcl::CollisionResult<S> &result); \
  extern template S distance<S>(                                                 \
      const FCLObjectPtr<S> &obj1, const FCLObjectPtr<S> &obj2,                  \
      const fcl::DistanceRequest<S> &request, fcl::DistanceResult<S> &result)

DECLARE_TEMPLATE_FCL_COMMON(float);
DECLARE_TEMPLATE_FCL_COMMON(double);

}  // namespace mplib::collision_detection::fcl
