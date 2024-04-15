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

  /// @brief Default constructor
  WorldCollisionResultTpl() {};

  /// Constructor with all members
  WorldCollisionResultTpl(const ::fcl::CollisionResult<S> &res,
                          const std::string &collision_type,
                          const std::string &object_name1,
                          const std::string &object_name2,
                          const std::string &link_name1, const std::string &link_name2)
      : res(res),
        collision_type(collision_type),
        object_name1(object_name1),
        object_name2(object_name2),
        link_name1(link_name1),
        link_name2(link_name2) {};

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

  /// @brief Default constructor
  WorldDistanceResultTpl() {};

  /// Constructor with all members
  WorldDistanceResultTpl(const ::fcl::DistanceResult<S> &res, S min_distance,
                         const std::string &distance_type,
                         const std::string &object_name1,
                         const std::string &object_name2, const std::string &link_name1,
                         const std::string &link_name2)
      : res(res),
        min_distance(min_distance),
        distance_type(distance_type),
        object_name1(object_name1),
        object_name2(object_name2),
        link_name1(link_name1),
        link_name2(link_name2) {};

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
