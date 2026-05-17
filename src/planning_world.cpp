#include "mplib/planning_world.h"

#include <memory>

#include "mplib/collision_detection/collision_matrix.h"
#include "mplib/collision_detection/fcl/fcl_utils.h"
#include "mplib/macros/assert.h"

namespace mplib {

template <typename S>
S getFCLContactMaxPenetration(const fcl::CollisionResult<S> &result) {
  S max_penetration = std::numeric_limits<S>::min();
  if (result.numContacts() == 0) return 1;

  for (size_t i = 0; i < result.numContacts(); ++i) {
    const auto &contact = result.getContact(i);
    max_penetration = std::max(max_penetration, contact.penetration_depth);
  }
  return max_penetration;
}

template <typename S>
PlanningWorldTpl<S>::PlanningWorldTpl(
    const std::vector<ArticulatedModelPtr> &articulations,
    const std::vector<FCLObjectPtr> &objects)
    : acm_(std::make_shared<AllowedCollisionMatrix>()) {
  for (const auto &art : articulations)
    planned_articulation_map_[art->getName()] = articulation_map_[art->getName()] = art;
  for (const auto &object : objects) object_map_[object->name] = object;
}

template <typename S>
std::vector<std::string> PlanningWorldTpl<S>::getArticulationNames() const {
  std::vector<std::string> names;
  for (const auto &pair : articulation_map_) names.push_back(pair.first);
  return names;
}

template <typename S>
std::vector<ArticulatedModelTplPtr<S>> PlanningWorldTpl<S>::getPlannedArticulations()
    const {
  std::vector<ArticulatedModelPtr> arts;
  for (const auto &pair : planned_articulation_map_) arts.push_back(pair.second);
  return arts;
}

template <typename S>
void PlanningWorldTpl<S>::addArticulation(const ArticulatedModelPtr &model,
                                          bool planned) {
  articulation_map_[model->getName()] = model;
  setArticulationPlanned(model->getName(), planned);
}

template <typename S>
bool PlanningWorldTpl<S>::removeArticulation(const std::string &name) {
  auto nh = articulation_map_.extract(name);
  if (nh.empty()) return false;
  planned_articulation_map_.erase(name);
  // Update acm_
  auto art_link_names = nh.mapped()->getUserLinkNames();
  acm_->removeEntry(art_link_names);
  acm_->removeDefaultEntry(art_link_names);
  return true;
}

template <typename S>
void PlanningWorldTpl<S>::setArticulationPlanned(const std::string &name,
                                                 bool planned) {
  auto art = articulation_map_.at(name);
  auto it = planned_articulation_map_.find(name);
  if (planned && it == planned_articulation_map_.end())
    planned_articulation_map_[name] = art;
  else if (!planned && it != planned_articulation_map_.end())
    planned_articulation_map_.erase(it);
}

template <typename S>
std::vector<std::string> PlanningWorldTpl<S>::getObjectNames() const {
  std::vector<std::string> names;
  for (const auto &pair : object_map_) names.push_back(pair.first);
  return names;
}

template <typename S>
void PlanningWorldTpl<S>::addObject(const std::string &name,
                                    const CollisionObjectPtr &collision_object) {
  addObject(
      std::make_shared<FCLObject>(name, Pose<S>(collision_object->getTransform()),
                                  std::vector<CollisionObjectPtr> {collision_object},
                                  std::vector<Pose<S>> {Pose<S>()}));
}

template <typename S>
void PlanningWorldTpl<S>::addPointCloud(const std::string &name,
                                        const MatrixX3<S> &vertices,
                                        double resolution,
                                        const Pose<S> &pose) {
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  for (const auto &row : vertices.rowwise())
    tree->updateNode(octomap::point3d(row(0), row(1), row(2)), true);
  auto cobject = std::make_shared<CollisionObject>(std::make_shared<fcl::OcTree<S>>(tree));
  addObject(
    std::make_shared<FCLObject>(
      name, pose, std::vector<CollisionObjectPtr> {cobject}, std::vector<Pose<S>> {Pose<S>()}
  ));
}

template <typename S>
bool PlanningWorldTpl<S>::removeObject(const std::string &name) {
  auto nh = object_map_.extract(name);
  if (nh.empty()) return false;
  attached_body_map_.erase(name);
  // Update acm_
  acm_->removeEntry(name);
  acm_->removeDefaultEntry(name);
  return true;
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const std::string &art_name, int link_id,
                                       const std::vector<std::string> &touch_links) {
  const auto T_world_obj = object_map_.at(name)->pose;
  const auto T_world_link =
      planned_articulation_map_.at(art_name)->getLinkGlobalPose(link_id);
  attachObject(name, art_name, link_id, Pose<S>(T_world_link.inverse() * T_world_obj),
               touch_links);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const std::string &art_name, int link_id) {
  const auto T_world_obj = object_map_.at(name)->pose;
  const auto T_world_link =
      planned_articulation_map_.at(art_name)->getLinkGlobalPose(link_id);
  attachObject(name, art_name, link_id, Pose<S>(T_world_link.inverse() * T_world_obj));
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const std::string &art_name, int link_id,
                                       const Pose<S> &pose,
                                       const std::vector<std::string> &touch_links) {
  auto obj = object_map_.at(name);
  auto nh = attached_body_map_.extract(name);
  auto body =
      std::make_shared<AttachedBody>(name, obj, planned_articulation_map_.at(art_name),
                                     link_id, pose.toIsometry(), touch_links);
  if (!nh.empty()) {
    // Update acm_ to disallow collision between name and previous touch_links
    acm_->removeEntry(name, nh.mapped()->getTouchLinks());
    nh.mapped() = body;
    attached_body_map_.insert(std::move(nh));
  } else
    attached_body_map_[name] = body;
  // Update acm_ to allow collision between name and touch_links
  acm_->setEntry(name, touch_links, true);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const std::string &art_name, int link_id,
                                       const Pose<S> &pose) {
  auto obj = object_map_.at(name);
  auto nh = attached_body_map_.extract(name);
  auto body = std::make_shared<AttachedBody>(
      name, obj, planned_articulation_map_.at(art_name), link_id, pose.toIsometry());
  if (!nh.empty()) {
    body->setTouchLinks(nh.mapped()->getTouchLinks());
    nh.mapped() = body;
    attached_body_map_.insert(std::move(nh));
  } else {
    attached_body_map_[name] = body;
    // Set touch_links to the name of self links colliding with object currently
    std::vector<std::string> touch_links;
    for (const auto &collision : checkSelfCollision())
      if (collision.link_name1 == name)
        touch_links.push_back(collision.link_name2);
      else if (collision.link_name2 == name)
        touch_links.push_back(collision.link_name1);
    body->setTouchLinks(touch_links);
    // Update acm_ to allow collision between name and touch_links
    acm_->setEntry(name, touch_links, true);
  }
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const CollisionGeometryPtr &obj_geom,
                                       const std::string &art_name, int link_id,
                                       const Pose<S> &pose,
                                       const std::vector<std::string> &touch_links) {
  removeObject(name);
  addObject(name, std::make_shared<CollisionObject>(obj_geom));
  attachObject(name, art_name, link_id, pose, touch_links);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const CollisionGeometryPtr &obj_geom,
                                       const std::string &art_name, int link_id,
                                       const Pose<S> &pose) {
  removeObject(name);
  addObject(name, std::make_shared<CollisionObject>(obj_geom));
  attachObject(name, art_name, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachSphere(S radius, const std::string &art_name,
                                       int link_id, const Pose<S> &pose) {
  // FIXME: Use link_name to avoid changes
  auto name = art_name + "_" + std::to_string(link_id) + "_sphere";
  attachObject(name, std::make_shared<fcl::Sphere<S>>(radius), art_name, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachBox(const Vector3<S> &size, const std::string &art_name,
                                    int link_id, const Pose<S> &pose) {
  // FIXME: Use link_name to avoid changes
  auto name = art_name + "_" + std::to_string(link_id) + "_box";
  attachObject(name, std::make_shared<fcl::Box<S>>(size), art_name, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachMesh(const std::string &mesh_path,
                                     const std::string &art_name, int link_id,
                                     const Pose<S> &pose, bool convex) {
  // FIXME: Use link_name to avoid changes
  auto name = art_name + "_" + std::to_string(link_id) + "_mesh";
  if (convex)
    attachObject(
        name,
        collision_detection::fcl::loadMeshAsConvex<S>(mesh_path, Vector3<S> {1, 1, 1}),
        art_name, link_id, pose);
  else
    attachObject(
        name,
        collision_detection::fcl::loadMeshAsBVH<S>(mesh_path, Vector3<S> {1, 1, 1}),
        art_name, link_id, pose);
}

/*
template <typename S>
void PlanningWorldTpl<S>::attachPointCloud(
  const std::string &name,
  const MatrixX3<S> &vertices,
  double resolution,
  const std::string &art_name,
  int link_id,
  const Pose<S> &pose) {
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  for (const auto &row : vertices.rowwise())
    tree->updateNode(octomap::point3d(row(0), row(1), row(2)), true);

  removeObject(name);
  auto obj = std::make_shared<CollisionObject>(std::make_shared<fcl::OcTree<S>>(tree));
  addObject(name, obj);
  attachObject(name, art_name, link_id, pose);
}
*/

template <typename S>
bool PlanningWorldTpl<S>::detachObject(const std::string &name, bool also_remove) {
  if (also_remove) {
    object_map_.erase(name);
    // Update acm_
    acm_->removeEntry(name);
    acm_->removeDefaultEntry(name);
  }

  auto nh = attached_body_map_.extract(name);
  if (nh.empty()) return false;
  // Update acm_ to disallow collision between name and touch_links
  acm_->removeEntry(name, nh.mapped()->getTouchLinks());
  return true;
}

template <typename S>
bool PlanningWorldTpl<S>::detachAllObjects(bool also_remove) {
  if (attached_body_map_.empty()) return false;
  std::vector<std::string> names;
  for (const auto &pair : attached_body_map_) names.push_back(pair.first);
  for (const auto &name : names) detachObject(name, also_remove);
  return true;
}

template <typename S>
void PlanningWorldTpl<S>::printAttachedBodyPose() const {
  for (const auto &[name, body] : attached_body_map_)
    std::cout << name << " global pose:\n"
              << Pose<S>(body->getGlobalPose()) << std::endl;
}

template <typename S>
void PlanningWorldTpl<S>::setQpos(const std::string &name,
                                  const VectorX<S> &qpos) const {
  articulation_map_.at(name)->setQpos(qpos);
}

template <typename S>
void PlanningWorldTpl<S>::setQposAll(const VectorX<S> &state) const {
  size_t i = 0;
  for (const auto &pair : planned_articulation_map_) {
    auto art = pair.second;
    auto n = art->getQposDim();
    auto qpos = state.segment(i, n);  // [i, i + n)
    ASSERT(static_cast<size_t>(qpos.size()) == n,
           "Bug with size " + std::to_string(qpos.size()) + " " + std::to_string(n));
    art->setQpos(qpos);
    i += n;
  }
  ASSERT(i == static_cast<size_t>(state.size()), "State dimension is not correct");
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkSelfCollision(
    const CollisionRequest &request) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  updateAttachedBodiesPose();

  // Collision involving planned articulation
  for (const auto &[art_name, art] : planned_articulation_map_) {
    auto fcl_model = art->getFCLModel();
    // Articulation self-collision
    const auto results = fcl_model->checkSelfCollision(request, acm_);
    ret.insert(ret.end(), results.begin(), results.end());

    // Collision among planned_articulation_map_
    for (const auto &[art_name2, art2] : planned_articulation_map_) {
      if (art_name == art_name2) continue;
      for (auto &result :
           fcl_model->checkCollisionWith(art2->getFCLModel(), request, acm_)) {
        result.collision_type = "self_self";
        ret.push_back(result);
      }
    }

    // Articulation collide with attached_body_map_
    for (const auto &[attached_body_name, attached_body] : attached_body_map_)
      for (auto &result :
           fcl_model->checkCollisionWith(attached_body->getObject(), request, acm_)) {
        result.collision_type = "self_attached";
        ret.push_back(result);
      }
  }

  // Collision among attached_body_map_
  for (auto it = attached_body_map_.begin(); it != attached_body_map_.end(); ++it)
    for (auto it2 = attached_body_map_.begin(); it2 != it; ++it2) {
      auto name1 = it->first, name2 = it2->first;
      if (auto type = acm_->getAllowedCollision(name1, name2);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        collision_detection::fcl::collide(it->second->getObject(),
                                          it2->second->getObject(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.collision_type = "attached_attached";
          tmp.object_name1 = name1;
          tmp.object_name2 = name2;
          tmp.link_name1 = name1;
          tmp.link_name2 = name2;
          tmp.max_penetration = getFCLContactMaxPenetration<S>(result);
          ret.push_back(tmp);
        }
      }
    }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkArticulationArticulationCollision(
  const ArticulatedModelPtr &art1, const ArticulatedModelPtr &art2,
  const CollisionRequest &request) const {
  const auto &fcl_model1 = art1->getFCLModel();
  const auto &fcl_model2 = art2->getFCLModel();
  auto results = fcl_model1->checkCollisionWith(fcl_model2, request, acm_);
  return results;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkArticulationObjectCollision(
  const ArticulatedModelPtr &art, const FCLObjectPtr &obj,
  const CollisionRequest &request) const {
  const auto &fcl_model1 = art->getFCLModel();
  auto results = fcl_model1->checkCollisionWith(obj, request, acm_);
  return results;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkObjectObjectCollision(
  const std::string &name1, const std::string &name2,
  const CollisionRequest &request) const {
  const auto &obj1 = object_map_.at(name1);
  const auto &obj2 = object_map_.at(name2);
  std::vector<WorldCollisionResultTpl<S>> results;
  CollisionResult result;
  if (auto type = acm_->getAllowedCollision(name1, name2);
      !type || type == collision_detection::AllowedCollision::NEVER) {
    collision_detection::fcl::collide(obj1, obj2, request, result);
    if (result.isCollision()) {
      WorldCollisionResult tmp;
      tmp.res = result;
      tmp.collision_type = "object_object";
      tmp.object_name1 = name1;
      tmp.object_name2 = name2;
      tmp.max_penetration = getFCLContactMaxPenetration<S>(result);
      results.push_back(tmp);
    }
  }
  return results;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkGeneralObjectPairCollision(
  const std::string &name1, const std::string &name2, const CollisionRequest &request) const {

  if (hasArticulation(name1) && hasArticulation(name2)) {
    const auto &art1 = getArticulation(name1);
    const auto &art2 = getArticulation(name2);
    return checkArticulationArticulationCollision(art1, art2, request);
  } else if (hasArticulation(name1) && hasObject(name2)) {
    const auto &art = getArticulation(name1);
    const auto &obj = getObject(name2);
    return checkArticulationObjectCollision(art, obj, request);
  } else if (hasObject(name1) && hasArticulation(name2)) {
    const auto &art = getArticulation(name2);
    const auto &obj = getObject(name1);
    return checkArticulationObjectCollision(art, obj, request);
  } else if (hasObject(name1) && hasObject(name2)) {
    return checkObjectObjectCollision(name1, name2, request);
  }

  std::vector<WorldCollisionResult> ret;
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkGeneralObjectCollision(
  const std::string &name, const CollisionRequest &request) const {

  std::vector<WorldCollisionResult> ret;

  for (const auto &[art_name, art] : articulation_map_) {
    if (name == art_name) continue;
    auto results = checkGeneralObjectPairCollision(name, art_name, request);
    ret.insert(ret.end(), results.begin(), results.end());
  }
  for (const auto &[obj_name, obj] : object_map_) {
    if (name == obj_name) continue;
    auto results = checkGeneralObjectPairCollision(name, obj_name, request);
    ret.insert(ret.end(), results.begin(), results.end());
  }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkRobotCollision(
    const CollisionRequest &request) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  updateAttachedBodiesPose();

  // Collect unplanned articulations, not attached scene objects
  std::vector<ArticulatedModelPtr> unplanned_articulations;
  std::vector<FCLObjectPtr> scene_objects;
  for (const auto &[name, art] : articulation_map_)
    if (planned_articulation_map_.find(name) == planned_articulation_map_.end())
      unplanned_articulations.push_back(art);
  for (const auto &[name, obj] : object_map_)
    if (attached_body_map_.find(name) == attached_body_map_.end())
      scene_objects.push_back(obj);

  // Collision involving planned articulation
  for (const auto &[art_name, art] : planned_articulation_map_) {
    auto fcl_model = art->getFCLModel();
    // Collision with unplanned articulation
    for (const auto &art2 : unplanned_articulations) {
      const auto results =
          fcl_model->checkCollisionWith(art2->getFCLModel(), request, acm_);
      ret.insert(ret.end(), results.begin(), results.end());
    }

    // Collision with scene objects
    for (const auto &scene_obj : scene_objects)
      for (auto &result : fcl_model->checkCollisionWith(scene_obj, request, acm_)) {
        result.collision_type = "self_sceneobject";
        ret.push_back(result);
      }
  }

  // Collision involving attached_body_map_
  for (const auto &[attached_body_name, attached_body] : attached_body_map_) {
    const auto attached_obj = attached_body->getObject();
    // Collision with unplanned articulation
    for (const auto &art2 : unplanned_articulations)
      for (auto &result :
           art2->getFCLModel()->checkCollisionWith(attached_obj, request, acm_)) {
        result.collision_type = "attached_articulation";
        ret.push_back(result);
      }

    // Collision with scene objects
    for (const auto &scene_obj : scene_objects)
      if (auto type = acm_->getAllowedCollision(attached_body_name, scene_obj->name);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        collision_detection::fcl::collide(attached_obj, scene_obj, request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.collision_type = "attached_sceneobject";
          tmp.object_name1 = attached_body_name;
          tmp.object_name2 = scene_obj->name;
          tmp.link_name1 = attached_body_name;
          tmp.link_name2 = scene_obj->name;
          tmp.max_penetration = getFCLContactMaxPenetration<S>(result);
          ret.push_back(tmp);
        }
      }
  }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkCollision(
    const CollisionRequest &request) const {
  auto ret1 = checkSelfCollision(request);
  const auto ret2 = checkRobotCollision(request);
  ret1.insert(ret1.end(), ret2.begin(), ret2.end());
  return ret1;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::checkSceneCollision(
  const std::vector<std::string> &scene_object_names,
  const CollisionRequest &request) const {

  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  std::vector<FCLObjectPtr> scene_objects;
  for (const auto &name : scene_object_names) {
    if (auto it = object_map_.find(name); it != object_map_.end())
      scene_objects.push_back(it->second);
  }
  for (size_t i = 0; i < scene_object_names.size(); i++) {
    for (size_t j = i + 1; j < scene_object_names.size(); j++) {
      if (auto type = acm_->getAllowedCollision(scene_object_names[i], scene_object_names[j]);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        collision_detection::fcl::collide(scene_objects[i], scene_objects[j], request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.collision_type = "sceneobject_sceneobject";
          tmp.object_name1 = scene_object_names[i];
          tmp.object_name2 = scene_object_names[j];
          tmp.max_penetration = getFCLContactMaxPenetration<S>(result);
          ret.push_back(tmp);
        }
      }
    }
  }
  return ret;
}

template <typename S>
WorldDistanceResultTpl<S> PlanningWorldTpl<S>::distanceSelf(
    const DistanceRequest &request) const {
  WorldDistanceResult ret;
  DistanceResult result;

  updateAttachedBodiesPose();

  // Minimum distance involving planned articulation
  for (const auto &[art_name, art] : planned_articulation_map_) {
    auto fcl_model = art->getFCLModel();
    // Articulation minimum distance to self-collision
    if (const auto &tmp = fcl_model->distanceSelf(request, acm_);
        tmp.min_distance < ret.min_distance)
      ret = tmp;

    // Minimum distance among planned_articulation_map_
    for (const auto &[art_name2, art2] : planned_articulation_map_) {
      if (art_name == art_name2) continue;
      if (const auto &tmp = fcl_model->distanceWith(art2->getFCLModel(), request, acm_);
          tmp.min_distance < ret.min_distance) {
        ret = tmp;
        ret.distance_type = "self_self";
      }
    }

    // Articulation minimum distance to attached_body_map_
    for (const auto &[attached_body_name, attached_body] : attached_body_map_)
      if (const auto &tmp =
              fcl_model->distanceWith(attached_body->getObject(), request, acm_);
          tmp.min_distance < ret.min_distance) {
        ret = tmp;
        ret.distance_type = "self_attached";
      }
  }

  // Minimum distance among attached_body_map_
  for (auto it = attached_body_map_.begin(); it != attached_body_map_.end(); ++it)
    for (auto it2 = attached_body_map_.begin(); it2 != it; ++it2) {
      auto name1 = it->first, name2 = it2->first;
      if (auto type = acm_->getAllowedCollision(name1, name2);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        collision_detection::fcl::distance(it->second->getObject(),
                                           it2->second->getObject(), request, result);
        if (result.min_distance < ret.min_distance) {
          ret.res = result;
          ret.min_distance = result.min_distance;
          ret.distance_type = "attached_attached";
          ret.object_name1 = name1;
          ret.object_name2 = name2;
          ret.link_name1 = name1;
          ret.link_name2 = name2;
        }
      }
    }
  return ret;
}

template <typename S>
WorldDistanceResultTpl<S> PlanningWorldTpl<S>::distanceRobot(
    const DistanceRequest &request) const {
  WorldDistanceResult ret;
  DistanceResult result;

  updateAttachedBodiesPose();

  // Collect unplanned articulations, not attached scene objects
  std::vector<ArticulatedModelPtr> unplanned_articulations;
  std::vector<FCLObjectPtr> scene_objects;
  for (const auto &[name, art] : articulation_map_)
    if (planned_articulation_map_.find(name) == planned_articulation_map_.end())
      unplanned_articulations.push_back(art);
  for (const auto &[name, obj] : object_map_)
    if (attached_body_map_.find(name) == attached_body_map_.end())
      scene_objects.push_back(obj);

  // Minimum distance involving planned articulation
  for (const auto &[art_name, art] : planned_articulation_map_) {
    auto fcl_model = art->getFCLModel();
    // Minimum distance to unplanned articulation
    for (const auto &art2 : unplanned_articulations)
      if (const auto &tmp = fcl_model->distanceWith(art2->getFCLModel(), request, acm_);
          tmp.min_distance < ret.min_distance)
        ret = tmp;

    // Minimum distance to scene objects
    for (const auto &scene_obj : scene_objects)
      if (const auto &tmp = fcl_model->distanceWith(scene_obj, request, acm_);
          tmp.min_distance < ret.min_distance) {
        ret = tmp;
        ret.distance_type = "self_sceneobject";
      }
  }

  // Minimum distance involving attached_body_map_
  for (const auto &[attached_body_name, attached_body] : attached_body_map_) {
    const auto attached_obj = attached_body->getObject();
    // Minimum distance to unplanned articulation
    for (const auto &art2 : unplanned_articulations)
      if (const auto &tmp =
              art2->getFCLModel()->distanceWith(attached_obj, request, acm_);
          tmp.min_distance < ret.min_distance) {
        ret = tmp;
        ret.distance_type = "attached_articulation";
      }

    // Minimum distance to scene objects
    for (const auto &scene_obj : scene_objects)
      if (auto type = acm_->getAllowedCollision(attached_body_name, scene_obj->name);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        collision_detection::fcl::distance(attached_obj, scene_obj, request, result);
        if (result.min_distance < ret.min_distance) {
          ret.res = result;
          ret.min_distance = result.min_distance;
          ret.distance_type = "attached_sceneobject";
          ret.object_name1 = attached_body_name;
          ret.object_name2 = scene_obj->name;
          ret.link_name1 = attached_body_name;
          ret.link_name2 = scene_obj->name;
        }
      }
  }
  return ret;
}

template <typename S>
WorldDistanceResultTpl<S> PlanningWorldTpl<S>::distance(
    const DistanceRequest &request) const {
  const auto ret1 = distanceSelf(request);
  const auto ret2 = distanceRobot(request);
  return ret1.min_distance < ret2.min_distance ? ret1 : ret2;
}

template <typename S>
std::vector<WorldDistanceResultTpl<S>> PlanningWorldTpl<S>::distanceScene(
    const std::vector<std::string> &scene_object_names,
    const DistanceRequest &request) const {

    std::vector<WorldDistanceResult> ret_list;
    WorldDistanceResult ret;
    DistanceResult result;

    std::vector<FCLObjectPtr> scene_objects;
    for (const auto &name : scene_object_names) {
        if (auto it = object_map_.find(name); it != object_map_.end())
            scene_objects.push_back(it->second);
    }
    for (size_t i = 0; i < scene_object_names.size(); i++) {
        for (size_t j = i + 1; j < scene_object_names.size(); j++) {
            if (auto type = acm_->getAllowedCollision(scene_object_names[i], scene_object_names[j]);
                !type || type == collision_detection::AllowedCollision::NEVER) {
                result.clear();
                collision_detection::fcl::distance(scene_objects[i], scene_objects[j], request, result);
                ret.res = result;
                ret.min_distance = result.min_distance;
                ret.distance_type = "sceneobject_sceneobject";
                ret.object_name1 = scene_object_names[i];
                ret.object_name2 = scene_object_names[j];
                ret_list.push_back(ret);
            }
        }
    }
    return ret_list;
}

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_PLANNING_WORLD(S) template class PlanningWorldTpl<S>

DEFINE_TEMPLATE_PLANNING_WORLD(float);
DEFINE_TEMPLATE_PLANNING_WORLD(double);

}  // namespace mplib
