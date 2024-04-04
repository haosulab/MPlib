#pragma once

#include <limits>
#include <string>

#include <fcl/narrowphase/collision_result.h>
#include <fcl/narrowphase/distance_result.h>

namespace mplib::collision_detection {

/// Result of the collision checking.
template <typename S>
struct WorldCollisionResultTpl {
  // TODO: Update with
  // https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1CollisionResult.html

  ::fcl::CollisionResult<S> res;  ///< the fcl CollisionResult
  std::string collision_type,     ///< type of the collision
      object_name1,               ///< name of the first object
      object_name2,               ///< name of the second object
      link_name1,                 ///< link name of the first object in collision
      link_name2;                 ///< link name of the second object in collision
};

/// Result of minimum distance-to-collision query.
template <typename S>
struct WorldDistanceResultTpl {
  // TODO: Update with
  // https://moveit.picknik.ai/main/api/html/structcollision__detection_1_1DistanceResult.html

  ::fcl::DistanceResult<S> res;  ///< the fcl DistanceResult
  /// minimum distance between the two objects
  S min_distance {std::numeric_limits<S>::max()};
  std::string distance_type,  ///< type of the distance result
      object_name1,           ///< name of the first object
      object_name2,           ///< name of the second object
      link_name1,             ///< link name of the first object
      link_name2;             ///< link name of the second object
};

}  // namespace mplib::collision_detection
