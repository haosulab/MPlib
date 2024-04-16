#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "mplib/collision_detection/collision_common.h"
#include "mplib/collision_detection/collision_matrix.h"
#include "mplib/collision_detection/fcl/collision_common.h"
#include "mplib/macros/class_forward.h"
#include "mplib/utils/pose.h"

namespace mplib::collision_detection::fcl {

// FCLModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(FCLModelTpl);

/**
 * FCL collision model of an articulation
 *
 * See https://github.com/flexible-collision-library/fcl
 */
template <typename S>
class FCLModelTpl {
  // Common type alias
  using CollisionRequest = fcl::CollisionRequest<S>;
  using CollisionResult = fcl::CollisionResult<S>;
  using DistanceRequest = fcl::DistanceRequest<S>;
  using DistanceResult = fcl::DistanceResult<S>;
  using WorldCollisionResult = WorldCollisionResultTpl<S>;
  using WorldDistanceResult = WorldDistanceResultTpl<S>;

 public:
  /**
   * Construct an FCL model from URDF and SRDF files.
   *
   * @param urdf_filename: path to URDF file, can be relative to the current working
   *  directory
   * @param convex: use convex decomposition for collision objects. Default: ``false``.
   * @param verbose: print debug information. Default: ``false``.
   */
  FCLModelTpl(const std::string &urdf_filename, bool convex = false,
              bool verbose = false);

  /**
   * Construct an FCL model from URDF and SRDF files.
   *
   * @param urdf_model: a urdf tree as urdf::ModelInterfaceSharedPtr
   * @param package_dir: path to replace package_dir for mesh files
   * @param convex: use convex decomposition for collision objects. Default: ``false``.
   * @param verbose: print debug information. Default: ``false``.
   */
  FCLModelTpl(const urdf::ModelInterfaceSharedPtr &urdf_model,
              const std::string &package_dir, bool convex = false,
              bool verbose = false);

  /**
   * Constructs a FCLModel from URDF string and collision links
   *
   * @param urdf_string: URDF string (without visual/collision elements for links)
   * @param collision_links: Vector of collision links as FCLObjectPtr.
   *    Format is: ``[FCLObjectPtr, ...]``.
   *    The collision objects are at the shape's local_pose.
   * @param verbose: print debug information. Default: ``false``.
   * @return: a unique_ptr to FCLModel
   */
  static std::unique_ptr<FCLModelTpl<S>> createFromURDFString(
      const std::string &urdf_string,
      const std::vector<FCLObjectPtr<S>> &collision_links, bool verbose = false);

  /**
   * Get name of the articulated model.
   *
   * @return: name of the articulated model
   */
  const std::string &getName() const { return name_; }

  /**
   * Set name of the articulated model.
   *
   * @param name: name of the articulated model
   */
  void setName(const std::string &name) { name_ = name; }

  /**
   * Get the collision objects of the FCL model.
   *
   * @return: all collision objects of the FCL model
   */
  const std::vector<FCLObjectPtr<S>> &getCollisionObjects() const {
    return collision_objects_;
  }

  const std::vector<std::string> &getCollisionLinkNames() const {
    return collision_link_names_;
  }

  /**
   * Get the collision pairs of the FCL model.
   *
   * @return: collision pairs of the FCL model. If the FCL model has N collision
   *  objects, the collision pairs is a list of N*(N-1)/2 pairs minus the disabled
   *  collision pairs
   */
  const std::vector<std::pair<size_t, size_t>> &getCollisionPairs() const {
    return collision_pairs_;
  }

  /// Print all collision pairs
  void printCollisionPairs() const;

  const std::vector<std::string> &getUserLinkNames() const { return user_link_names_; }

  const std::vector<size_t> &getCollisionLinkUserIndices() const {
    return collision_link_user_indices_;
  }

  /**
   * Set the link order of the FCL model.
   *
   * @param names: list of link names in the order that you want to set.
   */
  void setLinkOrder(const std::vector<std::string> &names);

  /**
   * Remove collision pairs from SRDF file.
   *
   * @param srdf_filename: path to SRDF file, can be relative to the current working
   *  directory
   */
  void removeCollisionPairsFromSRDF(const std::string &srdf_filename);

  /**
   * Remove collision pairs from SRDF string.
   *
   * @param srdf_string: SRDF string (only disable_collisions element)
   */
  void removeCollisionPairsFromSRDFString(const std::string &srdf_string);

  /**
   * Update the collision objects of the FCL model.
   *
   * @param link_poses: list of link poses in the order of the link order
   */
  void updateCollisionObjects(const std::vector<Pose<S>> &link_poses) const;

  /**
   * Check if the current state is in self-collision,
   * ignoring the distances between links that are allowed to always collide (as
   * specified by acm).
   *
   * @param acm: allowed collision matrix.
   * @return: ``true`` if any collision pair collides and ``false`` otherwise.
   */
  bool isStateColliding(const AllowedCollisionMatrixPtr &acm =
                            std::make_shared<AllowedCollisionMatrix>()) const {
    return checkSelfCollision(CollisionRequest(), acm).size() > 0;
  }

  /**
   * Check for self-collision in the current state and returns all found collisions,
   * ignoring the distances between links that are allowed to always collide (as
   * specified by acm).
   *
   * @param request: collision request
   * @param acm: allowed collision matrix.
   * @return: List of ``WorldCollisionResult`` objects. If empty, no self-collision.
   */
  std::vector<WorldCollisionResult> checkSelfCollision(
      const CollisionRequest &request = CollisionRequest(),
      const AllowedCollisionMatrixPtr &acm =
          std::make_shared<AllowedCollisionMatrix>()) const;

  /**
   * Check for collision in the current state with another ``FCLModel``,
   * ignoring the distances between links that are allowed to always
   * collide (as specified by acm).
   *
   * @param other: another ``FCLModel`` to check collision with
   * @param acm: allowed collision matrix.
   * @param request: collision request
   * @return: List of ``WorldCollisionResult`` objects. If empty, no collision.
   */
  std::vector<WorldCollisionResult> checkCollisionWith(
      const FCLModelTplPtr<S> &other,
      const CollisionRequest &request = CollisionRequest(),
      const AllowedCollisionMatrixPtr &acm =
          std::make_shared<AllowedCollisionMatrix>()) const;

  /**
   * Check for collision in the current state with an ``FCLObject``,
   * ignoring the distances between objects that are allowed to always
   * collide (as specified by acm).
   *
   * @param object: an ``FCLObject`` to check collision with
   * @param acm: allowed collision matrix.
   * @param request: collision request
   * @return: List of ``WorldCollisionResult`` objects. If empty, no collision.
   */
  std::vector<WorldCollisionResult> checkCollisionWith(
      const FCLObjectPtr<S> &object,
      const CollisionRequest &request = CollisionRequest(),
      const AllowedCollisionMatrixPtr &acm =
          std::make_shared<AllowedCollisionMatrix>()) const;

  /**
   * The minimum distance to self-collision given the robot in current state,
   * ignoring the distances between links that are allowed to always collide (as
   * specified by acm). Calls ``distanceSelf()``.
   *
   * @param acm: allowed collision matrix.
   * @return: minimum distance-to-self-collision
   */
  S distanceToSelfCollision(const AllowedCollisionMatrixPtr &acm =
                                std::make_shared<AllowedCollisionMatrix>()) const {
    return distanceSelf(DistanceRequest(), acm).min_distance;
  }

  /**
   * Get the minimum distance to self-collision given the robot in current state,
   * ignoring the distances between links that are allowed to always collide (as
   * specified by acm).
   *
   * @param request: distance request.
   * @param acm: allowed collision matrix.
   * @return: a ``WorldDistanceResult`` object
   */
  WorldDistanceResult distanceSelf(
      const DistanceRequest &request = DistanceRequest(),
      const AllowedCollisionMatrixPtr &acm =
          std::make_shared<AllowedCollisionMatrix>()) const;

  /**
   * The minimum distance to collision with another ``FCLModel`` given the robot in
   * current state, ignoring the distances between links that are allowed to always
   * collide (as specified by acm).
   *
   * @param other: another ``FCLModel`` to get minimum distance-to-collision with
   * @param acm: allowed collision matrix.
   * @return: minimum distance-to-collision with the other ``FCLModel``
   */
  S distanceToCollisionWith(const FCLModelTplPtr<S> &other,
                            const AllowedCollisionMatrixPtr &acm =
                                std::make_shared<AllowedCollisionMatrix>()) const {
    return distanceWith(other, DistanceRequest(), acm).min_distance;
  }

  /**
   * Get the minimum distance to collision with another ``FCLModel`` given the robot in
   * current state, ignoring the distances between links that are allowed to always
   * collide (as specified by acm).
   *
   * @param other: another ``FCLModel`` to get minimum distance-to-collision with
   * @param request: distance request.
   * @param acm: allowed collision matrix.
   * @return: a ``WorldDistanceResult`` object
   */
  WorldDistanceResult distanceWith(
      const FCLModelTplPtr<S> &other,
      const DistanceRequest &request = DistanceRequest(),
      const AllowedCollisionMatrixPtr &acm =
          std::make_shared<AllowedCollisionMatrix>()) const;

  /**
   * The minimum distance to collision with an ``FCLObject`` given the robot in
   * current state, ignoring the distances between objects that are allowed to always
   * collide (as specified by acm).
   *
   * @param object: an ``FCLObject`` to get minimum distance-to-collision with
   * @param acm: allowed collision matrix.
   * @return: minimum distance-to-collision with the ``FCLObject``
   */
  S distanceToCollisionWith(const FCLObjectPtr<S> &object,
                            const AllowedCollisionMatrixPtr &acm =
                                std::make_shared<AllowedCollisionMatrix>()) const {
    return distanceWith(object, DistanceRequest(), acm).min_distance;
  }

  /**
   * Get the minimum distance to collision with an ``FCLObject`` given the robot in
   * current state, ignoring the distances between objects that are allowed to always
   * collide (as specified by acm).
   *
   * @param object: an ``FCLObject`` to get minimum distance-to-collision with
   * @param request: distance request.
   * @param acm: allowed collision matrix.
   * @return: a ``WorldDistanceResult`` object
   */
  WorldDistanceResult distanceWith(
      const FCLObjectPtr<S> &object, const DistanceRequest &request = DistanceRequest(),
      const AllowedCollisionMatrixPtr &acm =
          std::make_shared<AllowedCollisionMatrix>()) const;

 private:
  void init(const urdf::ModelInterfaceSharedPtr &urdf_model,
            const std::string &package_dir_);

  void dfsParseTree(const urdf::LinkConstSharedPtr &link,
                    const std::string &parent_link_name);

  std::string name_;

  urdf::ModelInterfaceSharedPtr urdf_model_;
  std::string package_dir_;
  bool use_convex_ {};

  std::vector<FCLObjectPtr<S>> collision_objects_;
  std::vector<std::string> collision_link_names_;
  std::vector<std::pair<size_t, size_t>> collision_pairs_;

  std::vector<std::string> user_link_names_;
  std::vector<size_t> collision_link_user_indices_;

  bool verbose_ {};
};

// Common Type Alias ===================================================================
using FCLModelf = FCLModelTpl<float>;
using FCLModeld = FCLModelTpl<double>;
using FCLModelfPtr = FCLModelTplPtr<float>;
using FCLModeldPtr = FCLModelTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_FCL_MODEL(S) extern template class FCLModelTpl<S>

DECLARE_TEMPLATE_FCL_MODEL(float);
DECLARE_TEMPLATE_FCL_MODEL(double);

}  // namespace mplib::collision_detection::fcl
