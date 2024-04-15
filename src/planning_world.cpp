#include "mplib/planning_world.h"

#include <memory>

#include "mplib/collision_detection/collision_matrix.h"
#include "mplib/collision_detection/fcl/fcl_utils.h"
#include "mplib/macros/assert.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_PLANNING_WORLD(S) template class PlanningWorldTpl<S>

DEFINE_TEMPLATE_PLANNING_WORLD(float);
DEFINE_TEMPLATE_PLANNING_WORLD(double);

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
                                        double resolution) {
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  for (const auto &row : vertices.rowwise())
    tree->updateNode(octomap::point3d(row(0), row(1), row(2)), true);
  addObject(name,
            std::make_shared<CollisionObject>(std::make_shared<fcl::OcTree<S>>(tree)));
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
      planned_articulation_map_.at(art_name)->getPinocchioModel()->getLinkPose(link_id);
  attachObject(name, art_name, link_id, Pose<S>(T_world_link.inverse() * T_world_obj),
               touch_links);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const std::string &art_name, int link_id) {
  const auto T_world_obj = object_map_.at(name)->pose;
  const auto T_world_link =
      planned_articulation_map_.at(art_name)->getPinocchioModel()->getLinkPose(link_id);
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
    auto collisions = checkSelfCollision();
    for (const auto &collision : collisions)
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
                                       const CollisionGeometryPtr &p_geom,
                                       const std::string &art_name, int link_id,
                                       const Pose<S> &pose,
                                       const std::vector<std::string> &touch_links) {
  removeObject(name);
  addObject(name, std::make_shared<CollisionObject>(p_geom));
  attachObject(name, art_name, link_id, pose, touch_links);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const CollisionGeometryPtr &p_geom,
                                       const std::string &art_name, int link_id,
                                       const Pose<S> &pose) {
  removeObject(name);
  addObject(name, std::make_shared<CollisionObject>(p_geom));
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
          ret.push_back(tmp);
        }
      }
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

}  // namespace mplib
