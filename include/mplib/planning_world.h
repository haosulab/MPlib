#pragma once

#include <unordered_map>
#include <vector>

#include "articulated_model.h"
#include "color_printing.h"
#include "fcl_model.h"
#include "macros_utils.h"

/// Result of the collision checking.
template <typename DATATYPE>
struct WorldCollisionResultTpl {
  fcl::CollisionResult<DATATYPE> res;  ///< the fcl CollisionResult
  std::string collision_type,          ///< type of the collision
      object_name1,                    ///< name of the first object
      object_name2,                    ///< name of the second object
      link_name1,                      ///< link name of the first object in collision
      link_name2;                      ///< link name of the second object in collision
};

template <typename T>
using WorldCollisionResultTplPtr = std::shared_ptr<WorldCollisionResultTpl<T>>;

using WorldCollisionResultd = WorldCollisionResultTpl<double>;
using WorldCollisionResultf = WorldCollisionResultTpl<float>;
using WorldCollisionResultdPtr = WorldCollisionResultTplPtr<double>;
using WorldCollisionResultfPtr = WorldCollisionResultTplPtr<float>;

/// Planning world for collision checking
template <typename DATATYPE>
class PlanningWorldTpl {
 private:
  DEFINE_TEMPLATE_EIGEN(DATATYPE);
  using CollisionRequest = fcl::CollisionRequest<DATATYPE>;
  using CollisionResult = fcl::CollisionResult<DATATYPE>;

  using CollisionGeometry = fcl::CollisionGeometry<DATATYPE>;
  using CollisionGeometryPtr = std::shared_ptr<CollisionGeometry>;

  using CollisionObject = fcl::CollisionObject<DATATYPE>;
  using CollisionObjectPtr = std::shared_ptr<CollisionObject>;

  using DynamicAABBTreeCollisionManager =
      fcl::DynamicAABBTreeCollisionManager<DATATYPE>;
  using BroadPhaseCollisionManagerPtr =
      std::shared_ptr<fcl::BroadPhaseCollisionManager<DATATYPE>>;

  using ArticulatedModel = ArticulatedModelTpl<DATATYPE>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<DATATYPE>;

  using WorldCollisionResult = WorldCollisionResultTpl<DATATYPE>;
  using WorldCollisionResultPtr = WorldCollisionResultTplPtr<DATATYPE>;

  std::vector<ArticulatedModelPtr> articulations_;
  std::vector<std::string> articulation_names_;
  // std::vector<bool> articulation_flags;
  std::unordered_map<std::string, CollisionObjectPtr> normal_object_map_;
  int move_articulation_id_, attach_link_id_;
  CollisionObjectPtr point_cloud_, attached_tool_;
  bool has_point_cloud_, has_attach_;
  Transform3 attach_to_link_pose_;
  // BroadPhaseCollisionManagerPtr normal_manager;

 public:
  bool use_point_cloud_, use_attach_;  // expose to python

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

  // std::vector<bool> const &articulation_flags);

  /**
   * Get the list of articulated models.
   *
   * @return: list of articulated models
   */
  std::vector<ArticulatedModelPtr> getArticulations(void) { return articulations_; }

  /**
   * Get the list of non-articulated collision objects.
   *
   * @return: list of non-articulated collision objects
   */
  std::vector<CollisionObjectPtr> getNormalObjects(void) {
    std::vector<CollisionObjectPtr> ret;
    for (const auto &itm : normal_object_map_) ret.push_back(itm.second);
    return ret;
  }

  std::vector<std::string> &getArticulationNames() { return articulation_names_; }

  std::vector<std::string> getNormalObjectNames() {
    std::vector<std::string> ret;
    for (const auto &itm : normal_object_map_) ret.push_back(itm.first);
    return ret;
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

  void setMoveArticulationId(int id) { move_articulation_id_ = id; }

  int getMoveArticulationId() { return move_articulation_id_; }

  /**
   * Set whether to use point cloud for collision checking.
   *
   * @param use: whether to use point cloud
   */
  void setUsePointCloud(const bool &use) { use_point_cloud_ = use; }

  /**
   * Update the point cloud for collision checking.
   *
   * @param vertices: vertices of the point cloud
   * @param radius: radius of each point in the point cloud
   */
  void updatePointCloud(const Matrixx3 &vertices, const double &radius = 0.0);

  /**
   * Set whether to use attached tool for collision checking.
   *
   * @param use: whether to use attached tool
   */
  void setUseAttach(const bool &use) {
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
  void updateAttachedTool(CollisionGeometryPtr p_geom, int link_id,
                          const Vector7 &pose);

  /**
   * Add a sphere as the attached tool.
   *
   * @param radius: radius of the sphere
   * @param link_id: link id of the attached sphere
   * @param pose: pose of the attached sphere w.r.t. the link it's attached to.
   *              [x, y, z, qw, qx, qy, qz]
   */
  void updateAttachedSphere(DATATYPE radius, int link_id, const Vector7 &pose);

  /**
   * Add a box as the attached tool.
   *
   * @param size: size of the box, [size_x, size_y, size_z]
   * @param link_id: link id of the attached box
   * @param pose: pose of the attached box w.r.t. the link it's attached to.
   *              [x, y, z, qw, qx, qy, qz]
   */
  void updateAttachedBox(const Vector3 &size, int link_id, const Vector7 &pose);

  /**
   * Add a mesh as the attached tool.
   *
   * @param mesh_path: path to the mesh file
   * @param link_id: link id of the attached mesh
   * @param pose: pose of the attached mesh w.r.t. the link it's attached to.
   *              [x, y, z, qw, qx, qy, qz]
   */
  void updateAttachedMesh(const std::string &mesh_path, int link_id,
                          const Vector7 &pose);

  /// Print the pose of the attached tool.
  void printAttachedToolPose() {
    auto tmp1 = attached_tool_.get()->getTranslation();
    auto tmp2 = attached_tool_.get()->getRotation();
    print_info("Attached tool pose: ", tmp1.transpose(), " ", tmp2);
  }

  /**
   * Add an articulated model to the planning world.
   *
   * @param model: articulated model to be added
   * @param name: name of the articulated model
   */
  void addArticulation(const ArticulatedModelPtr &model,
                       const std::string &name) {  // bool const &planning = true) {
    articulations_.push_back(model);
    articulation_names_.push_back(name);
    // articulation_flags.push_back(planning);
  }

  /**
   * Add a list of articulated models to the planning world.
   *
   * @param models: list of articulated models to be added
   * @param names: list of names of the articulated models
   */
  void addArticulations(
      const std::vector<ArticulatedModelPtr> &models,
      const std::vector<std::string> &names) {  // std::vector<bool> const &planning) {
    articulations_.insert(articulations_.end(), models.begin(), models.end());
    articulation_names_.insert(articulation_names_.end(), names.begin(), names.end());
    // articulation_flags.insert(articulation_flags.end(), planning.begin(),
    // planning.end());
  }

  /**
   * Set the joint qpos of the articulated model.
   *
   * @param index: index of the articulated model
   * @param qpos: joint angles of the *movegroup only*
   */
  void setQpos(const int &index, const VectorX &qpos);

  /**
   * Set the joint qpos of all articulated models.
   *
   * @param qpos: joint angles of all the models (*movegroup only*)
   */
  void setQposAll(const VectorX &qpos);

  /**
   * Check collision in the planning world.
   *
   * @return: ``true`` if collision exists
   */
  bool collide();

  /**
   * Check collision between the articulated model and itself.
   *
   * @param index: index of the articulated model
   * @param request: collision request params. Can leave empty for default value
   * @return: List of WorldCollisionResult objects
   */
  std::vector<WorldCollisionResult> selfCollide(
      size_t index, const CollisionRequest &request = CollisionRequest());
  /**
   * Check collision between the articulated model and other objects.
   *
   * @param index: index of the articulated model
   * @param request: collision request params. Can leave empty for default value
   * @return: List of WorldCollisionResult objects
   */
  std::vector<WorldCollisionResult> collideWithOthers(
      size_t index, const CollisionRequest &request = CollisionRequest());

  /**
   * Check collision between the articulated model and all objects.
   *
   * @param index: index of the articulated model
   * @param request: collision request params. Can leave empty for default value
   * @return: List of WorldCollisionResult objects
   */
  std::vector<WorldCollisionResult> collideFull(
      size_t index, const CollisionRequest &request = CollisionRequest());
};

template <typename T>
using PlanningWorldTplPtr = std::shared_ptr<PlanningWorldTpl<T>>;

using PlanningWorldd = PlanningWorldTpl<double>;
using PlanningWorldf = PlanningWorldTpl<float>;
using PlanningWorlddPtr = PlanningWorldTplPtr<double>;
using PlanningWorldfPtr = PlanningWorldTplPtr<float>;
