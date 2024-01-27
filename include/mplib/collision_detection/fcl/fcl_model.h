#pragma once

#include <string>
#include <utility>
#include <vector>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <urdf_model/types.h>
#include <urdf_world/types.h>

#include "mplib/collision_detection/fcl/types.h"
#include "mplib/macros/class_forward.h"
#include "mplib/types.h"

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
   * Get the collision objects of the FCL model.
   *
   * @return: all collision objects of the FCL model
   */
  const std::vector<fcl::CollisionObjectPtr<S>> &getCollisionObjects() const {
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
   * Remove collision pairs from SRDF.
   *
   * @param srdf_filename: path to SRDF file, can be relative to the current working
   *  directory
   */
  void removeCollisionPairsFromSRDF(const std::string &srdf_filename);

  /**
   * Update the collision objects of the FCL model.
   *
   * @param link_poses: list of link poses in the order of the link order
   */
  void updateCollisionObjects(const std::vector<Vector7<S>> &link_pose) const;

  void updateCollisionObjects(const std::vector<Transform3<S>> &link_pose) const;

  /**
   * Perform self-collision checking.
   *
   * @param request: collision request
   * @return: ``true`` if any collision pair collides
   */
  bool collide(
      const fcl::CollisionRequest<S> &request = fcl::CollisionRequest<S>()) const;

  /**
   * Perform self-collision checking and returns all found collisions.
   *
   * @param request: collision request
   * @return: list of CollisionResult for each collision pair
   */
  std::vector<fcl::CollisionResult<S>> collideFull(
      const fcl::CollisionRequest<S> &request = fcl::CollisionRequest<S>(
          1, false, 1, false, true, fcl::GJKSolverType::GST_INDEP, 1e-6)) const;

 private:
  void init(const urdf::ModelInterfaceSharedPtr &urdf_model,
            const std::string &package_dir_);

  void dfsParseTree(const urdf::LinkConstSharedPtr &link,
                    const std::string &parent_link_name);

  urdf::ModelInterfaceSharedPtr urdf_model_;
  std::string package_dir_;
  bool use_convex_;

  std::vector<fcl::CollisionObjectPtr<S>> collision_objects_;
  std::vector<std::string> collision_link_names_;
  std::vector<std::string> parent_link_names_;
  std::vector<Transform3<S>> collision_origin2link_poses;
  std::vector<std::pair<size_t, size_t>> collision_pairs_;

  std::vector<std::string> user_link_names_;
  std::vector<size_t> collision_link_user_indices_;

  bool verbose_;
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
