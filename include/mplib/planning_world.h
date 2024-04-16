#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "mplib/collision_detection/collision_matrix.h"
#include "mplib/collision_detection/types.h"
#include "mplib/core/articulated_model.h"
#include "mplib/core/attached_body.h"
#include "mplib/macros/class_forward.h"
#include "mplib/types.h"
#include "mplib/utils/pose.h"

namespace mplib {

// PlanningWorldTplPtr
MPLIB_CLASS_TEMPLATE_FORWARD(PlanningWorldTpl);

/**
 * Planning world for collision checking
 *
 * Mimicking MoveIt2's ``planning_scene::PlanningScene``,
 * ``collision_detection::World``, ``moveit::core::RobotState``,
 * ``collision_detection::CollisionEnv``
 *
 * https://moveit.picknik.ai/main/api/html/classplanning__scene_1_1PlanningScene.html
 * https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1World.html
 * https://moveit.picknik.ai/main/api/html/classmoveit_1_1core_1_1RobotState.html
 * https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1CollisionEnv.html
 */
template <typename S>
class PlanningWorldTpl {
  // Common type alias
  using CollisionRequest = fcl::CollisionRequest<S>;
  using CollisionResult = fcl::CollisionResult<S>;
  using DistanceRequest = fcl::DistanceRequest<S>;
  using DistanceResult = fcl::DistanceResult<S>;
  using CollisionGeometryPtr = fcl::CollisionGeometryPtr<S>;
  using CollisionObject = fcl::CollisionObject<S>;
  using CollisionObjectPtr = fcl::CollisionObjectPtr<S>;
  using AllowedCollisionMatrix = collision_detection::AllowedCollisionMatrix;
  using AllowedCollisionMatrixPtr = collision_detection::AllowedCollisionMatrixPtr;
  using FCLObject = collision_detection::FCLObject<S>;
  using FCLObjectPtr = collision_detection::FCLObjectPtr<S>;
  // using DynamicAABBTreeCollisionManager = fcl::DynamicAABBTreeCollisionManager<S>;
  using BroadPhaseCollisionManagerPtr = fcl::BroadPhaseCollisionManagerPtr<S>;

  using WorldCollisionResult = WorldCollisionResultTpl<S>;
  using WorldDistanceResult = WorldDistanceResultTpl<S>;
  using ArticulatedModelPtr = ArticulatedModelTplPtr<S>;
  using AttachedBody = AttachedBodyTpl<S>;
  using AttachedBodyPtr = AttachedBodyTplPtr<S>;

 public:
  /**
   * Constructs a PlanningWorld with given (planned) articulations and objects
   *
   * @param articulations: list of planned articulated models
   * @param objects: list of non-articulated collision objects
   */
  PlanningWorldTpl(const std::vector<ArticulatedModelPtr> &articulations,
                   const std::vector<FCLObjectPtr> &objects = {});

  /// @brief Gets names of all articulations in world (unordered)
  std::vector<std::string> getArticulationNames() const;

  /// @brief Gets all planned articulations (ArticulatedModelPtr)
  std::vector<ArticulatedModelPtr> getPlannedArticulations() const;

  /**
   * Gets the articulation (ArticulatedModelPtr) with given name
   *
   * @param name: name of the articulated model
   * @return: the articulated model with given name or ``nullptr`` if not found.
   */
  ArticulatedModelPtr getArticulation(const std::string &name) const {
    auto it = articulation_map_.find(name);
    return it != articulation_map_.end() ? it->second : nullptr;
  }

  /**
   * Check whether the articulation with given name exists
   *
   * @param name: name of the articulated model
   * @return: ``true`` if exists, ``false`` otherwise.
   */
  bool hasArticulation(const std::string &name) const {
    return articulation_map_.find(name) != articulation_map_.end();
  }

  /**
   * Adds an articulation (ArticulatedModelPtr) to world
   *
   * @param model: articulated model to be added
   * @param planned: whether the articulation is being planned
   */
  void addArticulation(const ArticulatedModelPtr &model, bool planned = false);

  /**
   * Removes the articulation with given name if exists. Updates acm_
   *
   * @param name: name of the articulated model
   * @return: ``true`` if success, ``false`` if articulation with given name does not
   *  exist
   */
  bool removeArticulation(const std::string &name);

  /**
   * Check whether the articulation with given name is being planned
   *
   * @param name: name of the articulated model
   * @return: ``true`` if exists, ``false`` otherwise.
   */
  bool isArticulationPlanned(const std::string &name) const {
    return planned_articulation_map_.find(name) != planned_articulation_map_.end();
  }

  /**
   * Sets articulation with given name as being planned
   *
   * @param name: name of the articulated model
   * @param planned: whether the articulation is being planned
   * @throws std::out_of_range if the articulation with given name does not exist
   */
  void setArticulationPlanned(const std::string &name, bool planned);

  /// @brief Gets names of all objects in world (unordered)
  std::vector<std::string> getObjectNames() const;

  /**
   * Gets the non-articulated object (``FCLObjectPtr``) with given name
   *
   * @param name: name of the non-articulated object
   * @return: the object with given name or ``nullptr`` if not found.
   */
  FCLObjectPtr getObject(const std::string &name) const {
    auto it = object_map_.find(name);
    return it != object_map_.end() ? it->second : nullptr;
  }

  /**
   * Check whether the non-articulated object with given name exists
   *
   * @param name: name of the non-articulated object
   * @return: ``true`` if exists, ``false`` otherwise.
   */
  bool hasObject(const std::string &name) const {
    return object_map_.find(name) != object_map_.end();
  }

  /**
   * Adds an non-articulated object containing multiple collision objects
   * (``FCLObjectPtr``) to world
   *
   * @param fcl_obj: FCLObject to be added
   */
  void addObject(const FCLObjectPtr &fcl_obj) { object_map_[fcl_obj->name] = fcl_obj; }

  /**
   * Adds an non-articulated object (``CollisionObjectPtr``) with given name to world
   *
   * @param name: name of the collision object
   * @param collision_object: collision object to be added
   */
  void addObject(const std::string &name, const CollisionObjectPtr &collision_object);

  /**
   * Adds a point cloud as a collision object with given name to world
   *
   * @param name: name of the point cloud collision object
   * @param vertices: point cloud vertices matrix
   * @param resolution: resolution of the point in ``octomap::OcTree``
   */
  void addPointCloud(const std::string &name, const MatrixX3<S> &vertices,
                     double resolution = 0.01);

  /**
   * Removes (and detaches) the collision object with given name if exists.
   * Updates acm_
   *
   * @param name: name of the non-articulated collision object
   * @return: ``true`` if success, ``false`` if the non-articulated object
   * with given name does not exist
   */
  bool removeObject(const std::string &name);

  /**
   * Check whether the non-articulated object with given name is attached
   *
   * @param name: name of the non-articulated object
   * @return: ``true`` if it is attached, ``false`` otherwise.
   */
  bool isObjectAttached(const std::string &name) const {
    return attached_body_map_.find(name) != attached_body_map_.end();
  }

  /**
   * Gets the attached body (AttachedBodyPtr) with given name
   *
   * @param name: name of the attached body
   * @return: the attached body with given name or ``nullptr`` if not found.
   */
  AttachedBodyPtr getAttachedObject(const std::string &name) const {
    auto it = attached_body_map_.find(name);
    return it != attached_body_map_.end() ? it->second : nullptr;
  }

  /**
   * Attaches existing non-articulated object to specified link of articulation
   * at its current pose. If the object is currently attached, disallow collision
   * between the object and previous touch_links.
   *
   * Updates acm_ to allow collisions between attached object and touch_links.
   *
   * @param name: name of the non-articulated object to attach
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param touch_links: link names that the attached object touches
   * @throws std::out_of_range if non-articulated object with given name does not exist
   *  or if planned articulation with given name does not exist
   */
  void attachObject(const std::string &name, const std::string &art_name, int link_id,
                    const std::vector<std::string> &touch_links);

  /**
   * Attaches existing non-articulated object to specified link of articulation
   * at its current pose. If the object is not currently attached, automatically
   * sets touch_links as the name of self links that collide with the object
   * in the current state.
   *
   * Updates acm_ to allow collisions between attached object and touch_links.
   *
   * If the object is already attached, the touch_links of the attached object
   * is preserved and acm_ remains unchanged.
   *
   * @param name: name of the non-articulated object to attach
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @throws std::out_of_range if non-articulated object with given name does not exist
   *  or if planned articulation with given name does not exist
   */
  void attachObject(const std::string &name, const std::string &art_name, int link_id);

  /**
   * Attaches existing non-articulated object to specified link of articulation
   * at given pose. If the object is currently attached, disallow collision
   * between the object and previous touch_links.
   *
   * Updates acm_ to allow collisions between attached object and touch_links.
   *
   * @param name: name of the non-articulated object to attach
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   * @param touch_links: link names that the attached object touches
   * @throws std::out_of_range if non-articulated object with given name does not exist
   *  or if planned articulation with given name does not exist
   */
  void attachObject(const std::string &name, const std::string &art_name, int link_id,
                    const Pose<S> &pose, const std::vector<std::string> &touch_links);

  /**
   * Attaches existing non-articulated object to specified link of articulation
   * at given pose. If the object is not currently attached, automatically
   * sets touch_links as the name of self links that collide with the object
   * in the current state.
   *
   * Updates acm_ to allow collisions between attached object and touch_links.
   *
   * If the object is already attached, the touch_links of the attached object
   * is preserved and acm_ remains unchanged.
   *
   * @param name: name of the non-articulated object to attach
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   * @throws std::out_of_range if non-articulated object with given name does not exist
   *  or if planned articulation with given name does not exist
   */
  void attachObject(const std::string &name, const std::string &art_name, int link_id,
                    const Pose<S> &pose);

  /**
   * Attaches given object (w/ p_geom) to specified link of articulation at given pose.
   * This is done by removing the object and then adding and attaching object.
   * As a result, all previous acm_ entries with the object are removed
   *
   * @param name: name of the non-articulated object to attach
   * @param p_geom: pointer to a CollisionGeometry object
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   * @param touch_links: link names that the attached object touches
   */
  void attachObject(const std::string &name, const CollisionGeometryPtr &p_geom,
                    const std::string &art_name, int link_id, const Pose<S> &pose,
                    const std::vector<std::string> &touch_links);

  /**
   * Attaches given object (w/ p_geom) to specified link of articulation at given pose.
   * This is done by removing the object and then adding and attaching object.
   * As a result, all previous acm_ entries with the object are removed.
   * Automatically sets touch_links as the name of self links
   * that collide with the object in the current state (auto touch_links).
   *
   * @param name: name of the non-articulated object to attach
   * @param p_geom: pointer to a CollisionGeometry object
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   */
  void attachObject(const std::string &name, const CollisionGeometryPtr &p_geom,
                    const std::string &art_name, int link_id, const Pose<S> &pose);

  /**
   * Attaches given sphere to specified link of articulation (auto touch_links)
   *
   * @param radius: sphere radius
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   */
  void attachSphere(S radius, const std::string &art_name, int link_id,
                    const Pose<S> &pose);

  /**
   * Attaches given box to specified link of articulation (auto touch_links)
   *
   * @param size: box side length
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   */
  void attachBox(const Vector3<S> &size, const std::string &art_name, int link_id,
                 const Pose<S> &pose);

  /**
   * Attaches given mesh to specified link of articulation (auto touch_links)
   *
   * @param mesh_path: path to a mesh file
   * @param art_name: name of the planned articulation to attach to
   * @param link_id: index of the link of the planned articulation to attach to
   * @param pose: attached pose (relative pose from attached link to object)
   * @param convex: whether to load mesh as a convex mesh. Default: ``false``.
   */
  void attachMesh(const std::string &mesh_path, const std::string &art_name,
                  int link_id, const Pose<S> &pose, bool convex = false);

  /**
   * Detaches object with given name.
   * Updates acm_ to disallow collision between the object and touch_links.
   *
   * @param name: name of the non-articulated object to detach
   * @param also_remove: whether to also remove object from world
   * @return: ``true`` if success, ``false`` if the object with given name is not
   *  attached
   */
  bool detachObject(const std::string &name, bool also_remove = false);

  /// @brief Prints global pose of all attached bodies
  void printAttachedBodyPose() const;

  /**
   * Set qpos of articulation with given name
   *
   * @param name: name of the articulated model
   * @param qpos: joint angles of the *movegroup only*  // FIXME: double check
   */
  void setQpos(const std::string &name, const VectorX<S> &qpos) const;

  /// @brief Set qpos of all planned articulations
  void setQposAll(const VectorX<S> &state) const;

  /// @brief Get the current allowed collision matrix
  AllowedCollisionMatrixPtr getAllowedCollisionMatrix() const { return acm_; }

  /**
   * Check if the current state is in collision (with the environment or self
   * collision).
   *
   * @return: ``true`` if collision exists
   */
  bool isStateColliding() const {
    return checkCollision(CollisionRequest()).size() > 0;
  }

  /**
   * Check for self collision (including planned articulation self-collision,
   * planned articulation-attach collision, attach-attach collision)
   *
   * @param request: collision request params.
   * @return: List of ``WorldCollisionResult`` objects
   */
  std::vector<WorldCollisionResult> checkSelfCollision(
      const CollisionRequest &request = CollisionRequest()) const;

  /**
   * Check collision with other scene bodies in the world (planned articulations with
   * attached objects collide against unplanned articulations and scene objects)
   *
   * @param request: collision request params.
   * @return: List of ``WorldCollisionResult`` objects
   */
  std::vector<WorldCollisionResult> checkRobotCollision(
      const CollisionRequest &request = CollisionRequest()) const;

  /**
   * Check full collision (calls ``checkSelfCollision()`` and ``checkRobotCollision()``)
   *
   * @param request: collision request params.
   * @return: List of ``WorldCollisionResult`` objects
   */
  std::vector<WorldCollisionResult> checkCollision(
      const CollisionRequest &request = CollisionRequest()) const;

  /**
   * The minimum distance to self-collision given the robot in current state.
   * Calls ``distanceSelf()``.
   *
   * @return: minimum distance-to-self-collision
   */
  S distanceToSelfCollision() const { return distanceSelf().min_distance; }

  /**
   * Get the minimum distance to self-collision given the robot in current state
   *
   * @param request: distance request params.
   * @return: a ``WorldDistanceResult`` object
   */
  WorldDistanceResult distanceSelf(
      const DistanceRequest &request = DistanceRequest()) const;

  /**
   * The distance between the robot model at current state to the nearest collision
   * (ignoring self-collisions). Calls ``distanceRobot()``.
   *
   * @return: minimum distance-to-robot-collision
   */
  S distanceToRobotCollision() const { return distanceRobot().min_distance; }

  /**
   * Compute the minimum distance-to-collision between a robot and the world
   *
   * @param request: distance request params.
   * @return: a ``WorldDistanceResult`` object
   */
  WorldDistanceResult distanceRobot(
      const DistanceRequest &request = DistanceRequest()) const;

  /**
   * Compute the minimum distance-to-all-collision. Calls ``distance()``.
   *
   * Note that this is different from MoveIt2's
   * ``planning_scene::PlanningScene::distanceToCollision()`` where self-collisions are
   * ignored.
   *
   * @return: minimum distance-to-all-collision
   */
  S distanceToCollision() const { return distance().min_distance; }

  /**
   * Compute the minimum distance-to-all-collision (calls ``distanceSelf()`` and
   * ``distanceRobot()``)
   *
   * @param request: distance request params.
   * @return: a ``WorldDistanceResult`` object
   */
  WorldDistanceResult distance(
      const DistanceRequest &request = DistanceRequest()) const;

 private:
  std::unordered_map<std::string, ArticulatedModelPtr> articulation_map_;
  std::unordered_map<std::string, FCLObjectPtr> object_map_;

  // TODO: can planned_articulations_ be unordered_map? (setQposAll)
  std::map<std::string, ArticulatedModelPtr> planned_articulation_map_;
  std::unordered_map<std::string, AttachedBodyPtr> attached_body_map_;

  AllowedCollisionMatrixPtr acm_;

  // TODO: Switch to BroadPhaseCollision
  // BroadPhaseCollisionManagerPtr normal_manager;

  /// @brief Update attached bodies global pose using current state
  void updateAttachedBodiesPose() const {
    for (const auto &[name, attached_body] : attached_body_map_)
      attached_body->updatePose();
  }
};

// Common Type Alias ===================================================================
using PlanningWorldf = PlanningWorldTpl<float>;
using PlanningWorldd = PlanningWorldTpl<double>;
using PlanningWorldfPtr = PlanningWorldTplPtr<float>;
using PlanningWorlddPtr = PlanningWorldTplPtr<double>;

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_PLANNING_WORLD(S) extern template class PlanningWorldTpl<S>

DECLARE_TEMPLATE_PLANNING_WORLD(float);
DECLARE_TEMPLATE_PLANNING_WORLD(double);

}  // namespace mplib
