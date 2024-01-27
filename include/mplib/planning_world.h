#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "mplib/core/articulated_model.h"
#include "mplib/macros/class_forward.h"
#include "mplib/types.h"
#include "mplib/utils/color_printing.h"

namespace mplib {

// WorldCollisionResultTplPtr
MPLIB_STRUCT_TEMPLATE_FORWARD(WorldCollisionResultTpl);

/// Result of the collision checking.
template <typename S>
struct WorldCollisionResultTpl {
  fcl::CollisionResult<S> res;  ///< the fcl CollisionResult
  std::string collision_type,   ///< type of the collision
      object_name1,             ///< name of the first object
      object_name2,             ///< name of the second object
      link_name1,               ///< link name of the first object in collision
      link_name2;               ///< link name of the second object in collision
};

// Common Type Alias ===================================================================
using WorldCollisionResultf = WorldCollisionResultTpl<float>;
using WorldCollisionResultd = WorldCollisionResultTpl<double>;
using WorldCollisionResultfPtr = WorldCollisionResultTplPtr<float>;
using WorldCollisionResultdPtr = WorldCollisionResultTplPtr<double>;

// PlanningWorldTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(PlanningWorldTpl);

/// Planning world for collision checking
template <typename S>
class PlanningWorldTpl {
  // Common type alias
  using CollisionRequest = fcl::CollisionRequest<S>;
  using CollisionResult = fcl::CollisionResult<S>;
  using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
  using CollisionObject = fcl::CollisionObject<S>;
  using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
  // using DynamicAABBTreeCollisionManager = fcl::DynamicAABBTreeCollisionManager<S>;
  using BroadPhaseCollisionManagerPtr = fcl::BroadPhaseCollisionManagerPtr<S>;

  using WorldCollisionResult = WorldCollisionResultTpl<S>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;

 public:
  /**
   * Constructs a PlanningWorld with given articulations and normal objects
   *
   * @param articulations: list of articulated models
   * @param articulation_names: name of the articulated models
   * @param normal_objects: list of collision objects that are not articulated
   * @param normal_object_names: name of the normal objects
   * @param plan_articulation_id: id of the articulated model that is used for planning
   */
  PlanningWorldTpl(const std::vector<ArticulatedModelPtr> &articulations,
                   const std::vector<std::string> &articulation_names,
                   const std::vector<CollisionObjectPtr> &normal_objects,
                   const std::vector<std::string> &normal_object_names,
                   int plan_articulation_id = 0);

  /**
   * Get the list of articulated models.
   *
   * @return: list of articulated models
   */
  const std::vector<ArticulatedModelPtr> &getArticulations() const {
    return articulations_;
  }

  /**
   * Get the names of articulated models.
   *
   * @return: list of names of articulated models
   */
  const std::vector<std::string> &getArticulationNames() const {
    return articulation_names_;
  }

  /**
   * Get the list of non-articulated collision objects.
   *
   * @return: list of non-articulated collision objects
   */
  std::vector<CollisionObjectPtr> getNormalObjects() const {
    std::vector<CollisionObjectPtr> ret;
    for (const auto &itm : normal_object_map_) ret.push_back(itm.second);
    return ret;
  }

  /**
   * Get the names of non-articulated collision objects.
   *
   * @return: list of names of non-articulated collision objects
   */
  std::vector<std::string> getNormalObjectNames() const {
    std::vector<std::string> ret;
    for (const auto &itm : normal_object_map_) ret.push_back(itm.first);
    return ret;
  }

  int getMoveArticulationId() const { return move_articulation_id_; }

  void setMoveArticulationId(int id) { move_articulation_id_ = id; }

  /**
   * Add an articulated model to the planning world.
   *
   * @param model: articulated model to be added
   * @param name: name of the articulated model
   */
  void addArticulation(const ArticulatedModelPtr &model, const std::string &name) {
    articulations_.push_back(model);
    articulation_names_.push_back(name);
  }

  /**
   * Add a list of articulated models to the planning world.
   *
   * @param models: list of articulated models to be added
   * @param names: list of names of the articulated models
   */
  void addArticulations(const std::vector<ArticulatedModelPtr> &models,
                        const std::vector<std::string> &names) {
    articulations_.insert(articulations_.end(), models.begin(), models.end());
    articulation_names_.insert(articulation_names_.end(), names.begin(), names.end());
  }

  /**
   * Add a non-articulated collision object to the planning world.
   *
   * @param name: name of the non-articulated collision object
   * @param collision_object: the non-articulated collision object to be added
   */
  void setNormalObject(const std::string &name,
                       const CollisionObjectPtr &collision_object) {
    normal_object_map_[name] = collision_object;
  }

  /**
   * Remove am non-articulated object
   *
   * @param name: name of the non-articulated collision object
   * @return: ``true`` if the item exists and ``false`` otherwise
   */
  bool removeNormalObject(const std::string &name) {
    if (!normal_object_map_.count(name)) return false;
    normal_object_map_.erase(name);
    return true;
  }

  /**
   * Set whether to use point cloud for collision checking.
   *
   * @param use: whether to use point cloud
   */
  void setUsePointCloud(bool use) { use_point_cloud_ = use; }

  /**
   * Update the point cloud for collision checking.
   *
   * @param vertices: vertices of the point cloud
   * @param radius: radius of each point in the point cloud
   */
  void updatePointCloud(const MatrixX3<S> &vertices, double radius = 0.0);

  /**
   * Set whether to use attached tool for collision checking.
   *
   * @param use: whether to use attached tool
   */
  void setUseAttach(bool use) {
    use_attach_ = use;
    if (!use) removeAttach();
  }

  /**
   * Remove attach object so there won't be anything on the end effector when
   * ``use_attach`` is set to ``true`` again
   */
  void removeAttach() { has_attach_ = false; }

  /**
   * Attach or update the attached object
   *
   * @param p_geom: fcl collision geometry of the attached tool
   * @param link_id: id of the link to which the object is attached
   * @param pose: pose of the attached object w.r.t. the link it's attached to.
   *              [x, y, z, qw, qx, qy, qz]
   */
  void updateAttachedTool(const CollisionGeometryPtr &p_geom, int link_id,
                          const Vector7<S> &pose);

  /**
   * Add a box as the attached tool.
   *
   * @param size: size of the box, [size_x, size_y, size_z]
   * @param link_id: link id of the attached box
   * @param pose: pose of the attached box w.r.t. the link it's attached to.
   *              [x, y, z, qw, qx, qy, qz]
   */
  void updateAttachedBox(const Vector3<S> &size, int link_id, const Vector7<S> &pose);

  /**
   * Add a sphere as the attached tool.
   *
   * @param radius: radius of the sphere
   * @param link_id: link id of the attached sphere
   * @param pose: pose of the attached sphere w.r.t. the link it's attached to.
   *              [x, y, z, qw, qx, qy, qz]
   */
  void updateAttachedSphere(S radius, int link_id, const Vector7<S> &pose);

  /**
   * Add a mesh as the attached tool.
   *
   * @param mesh_path: path to the mesh file
   * @param link_id: link id of the attached mesh
   * @param pose: pose of the attached mesh w.r.t. the link it's attached to.
   *              [x, y, z, qw, qx, qy, qz]
   */
  void updateAttachedMesh(const std::string &mesh_path, int link_id,
                          const Vector7<S> &pose);

  /// Print the pose of the attached tool.
  void printAttachedToolPose() const {
    const auto tmp1 = attached_tool_.get()->getTranslation();
    const auto tmp2 = attached_tool_.get()->getRotation();
    print_info("Attached tool pose: ", tmp1.transpose(), " ", tmp2);
  }

  /**
   * Set the joint qpos of the articulated model.
   *
   * @param index: index of the articulated model
   * @param qpos: joint angles of the *movegroup only*
   */
  void setQpos(int index, const VectorX<S> &qpos) const;

  /**
   * Set the joint qpos of all articulated models.
   *
   * @param qpos: joint angles of all the models (*movegroup only*)
   */
  void setQposAll(const VectorX<S> &qpos) const;

  /**
   * Check collision in the planning world.
   *
   * @param request: collision request params. Can leave empty for default value
   * @return: ``true`` if collision exists
   */
  bool collide(const CollisionRequest &request = CollisionRequest()) const {
    return collideFull(0, request).size() > 0;
  }

  /**
   * Check collision between the articulated model and itself.
   *
   * @param index: index of the articulated model
   * @param request: collision request params. Can leave empty for default value
   * @return: List of WorldCollisionResult objects
   */
  std::vector<WorldCollisionResult> selfCollide(
      size_t index, const CollisionRequest &request = CollisionRequest()) const;
  /**
   * Check collision between the articulated model and other objects.
   *
   * @param index: index of the articulated model
   * @param request: collision request params. Can leave empty for default value
   * @return: List of WorldCollisionResult objects
   */
  std::vector<WorldCollisionResult> collideWithOthers(
      size_t index, const CollisionRequest &request = CollisionRequest()) const;

  /**
   * Check collision between the articulated model and all objects.
   *
   * @param index: index of the articulated model
   * @param request: collision request params. Can leave empty for default value
   * @return: List of WorldCollisionResult objects
   */
  std::vector<WorldCollisionResult> collideFull(
      size_t index, const CollisionRequest &request = CollisionRequest()) const;

  bool use_point_cloud_, use_attach_;  // expose to python

 private:
  std::vector<ArticulatedModelPtr> articulations_;
  std::vector<std::string> articulation_names_;

  std::unordered_map<std::string, CollisionObjectPtr> normal_object_map_;

  int move_articulation_id_, attach_link_id_;
  CollisionObjectPtr point_cloud_, attached_tool_;
  bool has_point_cloud_, has_attach_;
  Transform3<S> attach_to_link_pose_;

  // BroadPhaseCollisionManagerPtr normal_manager;
};

// Common Type Alias ===================================================================
using PlanningWorldf = PlanningWorldTpl<float>;
using PlanningWorldd = PlanningWorldTpl<double>;
using PlanningWorldfPtr = PlanningWorldTplPtr<float>;
using PlanningWorlddPtr = PlanningWorldTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_PLANNING_WORLD(S)           \
  extern template struct WorldCollisionResultTpl<S>; \
  extern template class PlanningWorldTpl<S>

DECLARE_TEMPLATE_PLANNING_WORLD(float);
DECLARE_TEMPLATE_PLANNING_WORLD(double);

}  // namespace mplib
