#pragma once
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "color_printing.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/common/types.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/gjk_solver_type.h"
#include "macros_utils.h"
#include "types.h"

namespace mplib::fcl {

// FCLModelTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(FCLModelTpl);

/**
 * FCL collision model of an articulation
 *
 * See https://github.com/flexible-collision-library/fcl
 */
template <typename S>
class FCLModelTpl {
 private:
  urdf::ModelInterfaceSharedPtr urdf_model_;

  std::vector<CollisionObjectPtr<S>> collision_objects_;
  std::vector<Transform3<S>> collision_origin2link_poses;
  std::vector<std::string> collision_link_names_;
  std::vector<std::string> parent_link_names_;
  std::vector<std::pair<size_t, size_t>> collision_pairs_;

  std::vector<std::string> user_link_names_;
  std::vector<size_t> collision_link_user_indices_;
  std::string package_dir_;
  bool have_link_order_, use_convex_, verbose_;

  void init(const urdf::ModelInterfaceSharedPtr &urdfTree,
            const std::string &package_dir_);
  void dfs_parse_tree(const urdf::LinkConstSharedPtr &link, std::string);

 public:
  FCLModelTpl(const urdf::ModelInterfaceSharedPtr &urdfTree,
              const std::string &package_dir, const bool &verbose = true,
              const bool &convex = false);

  /**
   * Construct an FCL model from URDF and SRDF files.
   *
   * @param urdf_filename: path to URDF file, can be relative to the current working
   *  directory
   * @param verbose: print debug information
   * @param convex: use convex decomposition for collision objects
   */
  FCLModelTpl(const std::string &urdf_filename, const bool &verbose = true,
              const bool &convex = false);

  /**
   * Get the collision pairs of the FCL model.
   *
   * @return: collision pairs of the FCL model. If the FCL model has N collision
   *  objects, the collision pairs is a list of N*(N-1)/2 pairs minus the disabled
   *  collision pairs
   */
  std::vector<std::pair<size_t, size_t>> &getCollisionPairs() {
    return collision_pairs_;
  }

  /**
   * Get the collision objects of the FCL model.
   *
   * @return: all collision objects of the FCL model
   */
  std::vector<CollisionObjectPtr<S>> &getCollisionObjects() {
    return collision_objects_;
  }

  std::vector<std::string> getCollisionLinkNames() { return collision_link_names_; }

  std::vector<std::string> getUserLinkNames() { return user_link_names_; }

  std::vector<size_t> getCollisionLinkUserIndices() {
    return collision_link_user_indices_;
  }

  /**
   * Set the link order of the FCL model.
   *
   * @param names: list of link names in the order that you want to set.
   */
  void setLinkOrder(const std::vector<std::string> &names);

  void printCollisionPairs(void);

  /**
   * Remove collision pairs from SRDF.
   *
   * @param srdf_filename: path to SRDF file, can be relative to the current working
   *  directory
   */
  void removeCollisionPairsFromSrdf(const std::string &srdf_filename);

  void updateCollisionObjects(const std::vector<Transform3<S>> &link_pose);

  /**
   * Update the collision objects of the FCL model.
   *
   * @param link_poses: list of link poses in the order of the link order
   */
  void updateCollisionObjects(const std::vector<Vector7<S>> &link_pose);

  /**
   * Perform collision checking.
   *
   * @param request: collision request
   * @return: ``true`` if collision happens
   */
  bool collide(const CollisionRequest<S> &request = CollisionRequest<S>());

  std::vector<CollisionResult<S>> collideFull(
      const CollisionRequest<S> &request = CollisionRequest<S>(1, false, 1, false, true,
                                                               GJKSolverType::GST_INDEP,
                                                               1e-6));
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

}  // namespace mplib::fcl
