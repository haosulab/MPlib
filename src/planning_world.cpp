#include "mplib/planning_world.h"

#include <memory>

#include "mplib/collision_detection/collision_matrix.h"
#include "mplib/collision_detection/fcl/fcl_utils.h"
#include "mplib/macros/assert.h"
#include "mplib/utils/conversion.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_PLANNING_WORLD(S) template class PlanningWorldTpl<S>

DEFINE_TEMPLATE_PLANNING_WORLD(float);
DEFINE_TEMPLATE_PLANNING_WORLD(double);

template <typename S>
PlanningWorldTpl<S>::PlanningWorldTpl(
    const std::vector<ArticulatedModelPtr> &articulations,
    const std::vector<std::string> &articulation_names,
    const std::vector<CollisionObjectPtr> &normal_objects,
    const std::vector<std::string> &normal_object_names)
    : acm_(std::make_shared<AllowedCollisionMatrix>()) {
  ASSERT(articulations.size() == articulation_names.size(),
         "articulations and articulation_names should have the same size");
  ASSERT(normal_objects.size() == normal_object_names.size(),
         "normal_objects and normal_object_names should have the same size");
  // TODO(merge): remove articulation_names, and normal_object_names
  for (size_t i = 0; i < articulations.size(); i++) {
    articulations[i]->setName(articulation_names[i]);
    articulation_map_[articulation_names[i]] = articulations[i];
    planned_articulation_map_[articulation_names[i]] = articulations[i];
  }
  for (size_t i = 0; i < normal_objects.size(); i++) {
    normal_object_map_[normal_object_names[i]] = normal_objects[i];
  }
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
void PlanningWorldTpl<S>::addArticulation(const std::string &name,
                                          const ArticulatedModelPtr &model,
                                          bool planned) {
  model->setName(name);
  articulation_map_[name] = model;
  setArticulationPlanned(name, planned);
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
std::vector<std::string> PlanningWorldTpl<S>::getNormalObjectNames() const {
  std::vector<std::string> names;
  for (const auto &pair : normal_object_map_) names.push_back(pair.first);
  return names;
}

template <typename S>
void PlanningWorldTpl<S>::addPointCloud(const std::string &name,
                                        const MatrixX3<S> &vertices,
                                        double resolution) {
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  for (const auto &row : vertices.rowwise())
    tree->updateNode(octomap::point3d(row(0), row(1), row(2)), true);
  auto obj = std::make_shared<CollisionObject>(std::make_shared<fcl::OcTree<S>>(tree));
  addNormalObject(name, obj);
}

template <typename S>
bool PlanningWorldTpl<S>::removeNormalObject(const std::string &name) {
  auto nh = normal_object_map_.extract(name);
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
  const auto T_world_obj = normal_object_map_.at(name)->getTransform();
  const auto T_world_link = toIsometry(
      planned_articulation_map_.at(art_name)->getPinocchioModel()->getLinkPose(
          link_id));
  attachObject(name, art_name, link_id, toPoseVec(T_world_link.inverse() * T_world_obj),
               touch_links);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const std::string &art_name, int link_id) {
  const auto T_world_obj = normal_object_map_.at(name)->getTransform();
  const auto T_world_link = toIsometry(
      planned_articulation_map_.at(art_name)->getPinocchioModel()->getLinkPose(
          link_id));
  attachObject(name, art_name, link_id,
               toPoseVec(T_world_link.inverse() * T_world_obj));
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const std::string &art_name, int link_id,
                                       const Vector7<S> &pose,
                                       const std::vector<std::string> &touch_links) {
  auto obj = normal_object_map_.at(name);
  auto nh = attached_body_map_.extract(name);
  auto body =
      std::make_shared<AttachedBody>(name, obj, planned_articulation_map_.at(art_name),
                                     link_id, toIsometry(pose), touch_links);
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
                                       const Vector7<S> &pose) {
  auto obj = normal_object_map_.at(name);
  auto nh = attached_body_map_.extract(name);
  auto body = std::make_shared<AttachedBody>(
      name, obj, planned_articulation_map_.at(art_name), link_id, toIsometry(pose));
  if (!nh.empty()) {
    body->setTouchLinks(nh.mapped()->getTouchLinks());
    nh.mapped() = body;
    attached_body_map_.insert(std::move(nh));
  } else {
    attached_body_map_[name] = body;
    // Set touch_links to the name of self links colliding with object currently
    std::vector<std::string> touch_links;
    auto collisions = selfCollide();
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
                                       const Vector7<S> &pose,
                                       const std::vector<std::string> &touch_links) {
  removeNormalObject(name);
  addNormalObject(name, std::make_shared<CollisionObject>(p_geom));
  attachObject(name, art_name, link_id, pose, touch_links);
}

template <typename S>
void PlanningWorldTpl<S>::attachObject(const std::string &name,
                                       const CollisionGeometryPtr &p_geom,
                                       const std::string &art_name, int link_id,
                                       const Vector7<S> &pose) {
  removeNormalObject(name);
  addNormalObject(name, std::make_shared<CollisionObject>(p_geom));
  attachObject(name, art_name, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachSphere(S radius, const std::string &art_name,
                                       int link_id, const Vector7<S> &pose) {
  // FIXME: Use link_name to avoid changes
  auto name = art_name + "_" + std::to_string(link_id) + "_sphere";
  attachObject(name, std::make_shared<fcl::Sphere<S>>(radius), art_name, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachBox(const Vector3<S> &size, const std::string &art_name,
                                    int link_id, const Vector7<S> &pose) {
  // FIXME: Use link_name to avoid changes
  auto name = art_name + "_" + std::to_string(link_id) + "_box";
  attachObject(name, std::make_shared<fcl::Box<S>>(size), art_name, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::attachMesh(const std::string &mesh_path,
                                     const std::string &art_name, int link_id,
                                     const Vector7<S> &pose) {
  // FIXME: Use link_name to avoid changes
  auto name = art_name + "_" + std::to_string(link_id) + "_mesh";
  attachObject(
      name, collision_detection::fcl::loadMeshAsBVH<S>(mesh_path, Vector3<S> {1, 1, 1}),
      art_name, link_id, pose);
}

template <typename S>
bool PlanningWorldTpl<S>::detachObject(const std::string &name, bool also_remove) {
  if (also_remove) {
    normal_object_map_.erase(name);
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
              << body->getGlobalPose().matrix() << std::endl;
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
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::filterCollisions(
    const std::vector<WorldCollisionResultTpl<S>> &collisions) const {
  std::vector<WorldCollisionResult> ret;
  for (const auto &collision : collisions)
    if (auto type =
            acm_->getAllowedCollision(collision.link_name1, collision.link_name2);
        !type || type == collision_detection::AllowedCollision::NEVER)
      ret.push_back(collision);
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::selfCollide(
    const CollisionRequest &request) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  updateAttachedBodiesPose();

  // Collision involving planned articulation
  for (const auto &[art_name, art] : planned_articulation_map_) {
    auto fcl_model = art->getFCLModel();
    auto col_objs = fcl_model->getCollisionObjects();
    auto col_link_names = fcl_model->getCollisionLinkNames();
    auto col_pairs = fcl_model->getCollisionPairs();

    // Articulation self-collision
    auto results = fcl_model->collideFull(request);
    for (size_t i = 0; i < results.size(); i++)
      if (results[i].isCollision()) {
        WorldCollisionResult tmp;
        auto x = col_pairs[i].first, y = col_pairs[i].second;
        tmp.res = results[i];
        tmp.collision_type = "self";
        tmp.object_name1 = art_name;
        tmp.object_name2 = art_name;
        tmp.link_name1 = col_link_names[x];
        tmp.link_name2 = col_link_names[y];
        ret.push_back(tmp);
      }

    // Collision among planned_articulations_
    for (const auto &[art_name2, art2] : planned_articulation_map_) {
      if (art_name == art_name2) break;
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs.size(); i++)
        for (size_t j = 0; j < col_objs2.size(); j++) {
          result.clear();
          ::fcl::collide(col_objs[i].get(), col_objs2[j].get(), request, result);
          if (result.isCollision()) {
            WorldCollisionResult tmp;
            tmp.res = result;
            tmp.collision_type = "self_articulation";
            tmp.object_name1 = art_name;
            tmp.object_name2 = art_name2;
            tmp.link_name1 = col_link_names[i];
            tmp.link_name2 = col_link_names2[j];
            ret.push_back(tmp);
          }
        }
    }

    // Articulation collide with attached_bodies_
    for (const auto &[attached_body_name, attached_body] : attached_body_map_) {
      auto attached_obj = attached_body->getObject();
      for (size_t i = 0; i < col_objs.size(); i++) {
        result.clear();
        ::fcl::collide(attached_obj.get(), col_objs[i].get(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.collision_type = "self_attach";
          tmp.object_name1 = art_name;
          tmp.object_name2 = attached_body_name;
          tmp.link_name1 = col_link_names[i];
          tmp.link_name2 = attached_body_name;
          ret.push_back(tmp);
        }
      }
    }
  }

  // Collision among attached_bodies_
  for (auto it = attached_body_map_.begin(); it != attached_body_map_.end(); ++it)
    for (auto it2 = attached_body_map_.begin(); it2 != it; ++it2) {
      result.clear();
      ::fcl::collide(it->second->getObject().get(), it2->second->getObject().get(),
                     request, result);
      if (result.isCollision()) {
        auto name1 = it->first, name2 = it2->first;
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.collision_type = "attach_attach";
        tmp.object_name1 = name1;
        tmp.object_name2 = name2;
        tmp.link_name1 = name1;
        tmp.link_name2 = name2;
        ret.push_back(tmp);
      }
    }
  return filterCollisions(ret);
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::collideWithOthers(
    const CollisionRequest &request) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  updateAttachedBodiesPose();

  // Collect unplanned articulations, not attached scene objects
  std::vector<ArticulatedModelPtr> unplanned_articulations;
  std::unordered_map<std::string, CollisionObjectPtr> scene_objects;
  for (const auto &[name, art] : articulation_map_)
    if (planned_articulation_map_.find(name) == planned_articulation_map_.end())
      unplanned_articulations.push_back(art);
  for (const auto &[name, obj] : normal_object_map_)
    if (attached_body_map_.find(name) == attached_body_map_.end())
      scene_objects[name] = obj;

  // Collision involving planned articulation
  for (const auto &[art_name, art] : planned_articulation_map_) {
    auto fcl_model = art->getFCLModel();
    auto col_objs = fcl_model->getCollisionObjects();
    auto col_link_names = fcl_model->getCollisionLinkNames();

    // Collision with unplanned articulation
    for (const auto &art2 : unplanned_articulations) {
      auto art_name2 = art2->getName();
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs.size(); i++)
        for (size_t j = 0; j < col_objs2.size(); j++) {
          result.clear();
          ::fcl::collide(col_objs[i].get(), col_objs2[j].get(), request, result);
          if (result.isCollision()) {
            WorldCollisionResult tmp;
            tmp.res = result;
            tmp.collision_type = "articulation_articulation";
            tmp.object_name1 = art_name;
            tmp.object_name2 = art_name2;
            tmp.link_name1 = col_link_names[i];
            tmp.link_name2 = col_link_names2[j];
            ret.push_back(tmp);
          }
        }
    }

    // Collision with scene objects
    for (const auto &[name, obj] : scene_objects)
      for (size_t i = 0; i < col_objs.size(); i++) {
        result.clear();
        ::fcl::collide(col_objs[i].get(), obj.get(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.collision_type = "articulation_sceneobject";
          tmp.object_name1 = art_name;
          tmp.object_name2 = name;
          tmp.link_name1 = col_link_names[i];
          tmp.link_name2 = name;
          ret.push_back(tmp);
        }
      }
  }

  // Collision involving attached_bodies_
  for (const auto &[attached_body_name, attached_body] : attached_body_map_) {
    auto attached_obj = attached_body->getObject();

    // Collision with unplanned articulation
    for (const auto &art2 : unplanned_articulations) {
      auto art_name2 = art2->getName();
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs2.size(); i++) {
        result.clear();
        ::fcl::collide(attached_obj.get(), col_objs2[i].get(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.collision_type = "attach_articulation";
          tmp.object_name1 = attached_body_name;
          tmp.object_name2 = art_name2;
          tmp.link_name1 = attached_body_name;
          tmp.link_name2 = col_link_names2[i];
          ret.push_back(tmp);
        }
      }
    }

    // Collision with scene objects
    for (const auto &[name, obj] : scene_objects) {
      result.clear();
      ::fcl::collide(attached_obj.get(), obj.get(), request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.collision_type = "attach_sceneobject";
        tmp.object_name1 = attached_body_name;
        tmp.object_name2 = name;
        tmp.link_name1 = attached_body_name;
        tmp.link_name2 = name;
        ret.push_back(tmp);
      }
    }
  }
  return filterCollisions(ret);
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::collideFull(
    const CollisionRequest &request) const {
  auto ret1 = selfCollide(request);
  auto ret2 = collideWithOthers(request);
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
    auto col_objs = fcl_model->getCollisionObjects();
    auto col_link_names = fcl_model->getCollisionLinkNames();
    auto col_pairs = fcl_model->getCollisionPairs();

    // Articulation minimum distance to self-collision
    for (const auto &[x, y] : col_pairs)
      if (auto type = acm_->getAllowedCollision(col_link_names[x], col_link_names[y]);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        ::fcl::distance(col_objs[x].get(), col_objs[y].get(), request, result);
        if (result.min_distance < ret.min_distance) {
          ret.res = result;
          ret.min_distance = result.min_distance;
          ret.distance_type = "self";
          ret.object_name1 = art_name;
          ret.object_name2 = art_name;
          ret.link_name1 = col_link_names[x];
          ret.link_name2 = col_link_names[y];
        }
      }

    // Minimum distance among planned_articulations_
    for (const auto &[art_name2, art2] : planned_articulation_map_) {
      if (art_name == art_name2) break;
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs.size(); i++)
        for (size_t j = 0; j < col_objs2.size(); j++)
          if (auto type =
                  acm_->getAllowedCollision(col_link_names[i], col_link_names2[j]);
              !type || type == collision_detection::AllowedCollision::NEVER) {
            result.clear();
            ::fcl::distance(col_objs[i].get(), col_objs2[j].get(), request, result);
            if (result.min_distance < ret.min_distance) {
              ret.res = result;
              ret.min_distance = result.min_distance;
              ret.distance_type = "self_articulation";
              ret.object_name1 = art_name;
              ret.object_name2 = art_name2;
              ret.link_name1 = col_link_names[i];
              ret.link_name2 = col_link_names2[j];
            }
          }
    }

    // Articulation minimum distance to attached_body_map_
    for (const auto &[attached_body_name, attached_body] : attached_body_map_) {
      auto attached_obj = attached_body->getObject();
      for (size_t i = 0; i < col_objs.size(); i++) {
        if (auto type =
                acm_->getAllowedCollision(col_link_names[i], attached_body_name);
            !type || type == collision_detection::AllowedCollision::NEVER) {
          result.clear();
          ::fcl::distance(attached_obj.get(), col_objs[i].get(), request, result);
          if (result.min_distance < ret.min_distance) {
            ret.res = result;
            ret.min_distance = result.min_distance;
            ret.distance_type = "self_attach";
            ret.object_name1 = art_name;
            ret.object_name2 = attached_body_name;
            ret.link_name1 = col_link_names[i];
            ret.link_name2 = attached_body_name;
          }
        }
      }
    }
  }

  // Minimum distance among attached_body_map_
  for (auto it = attached_body_map_.begin(); it != attached_body_map_.end(); ++it)
    for (auto it2 = attached_body_map_.begin(); it2 != it; ++it2) {
      auto name1 = it->first, name2 = it2->first;
      if (auto type = acm_->getAllowedCollision(name1, name2);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        ::fcl::distance(it->second->getObject().get(), it2->second->getObject().get(),
                        request, result);
        if (result.min_distance < ret.min_distance) {
          ret.res = result;
          ret.min_distance = result.min_distance;
          ret.distance_type = "attach_attach";
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
WorldDistanceResultTpl<S> PlanningWorldTpl<S>::distanceOthers(
    const DistanceRequest &request) const {
  WorldDistanceResult ret;
  DistanceResult result;

  updateAttachedBodiesPose();

  // Collect unplanned articulations, not attached scene objects
  std::vector<ArticulatedModelPtr> unplanned_articulations;
  std::unordered_map<std::string, CollisionObjectPtr> scene_objects;
  for (const auto &[name, art] : articulation_map_)
    if (planned_articulation_map_.find(name) == planned_articulation_map_.end())
      unplanned_articulations.push_back(art);
  for (const auto &[name, obj] : normal_object_map_)
    if (attached_body_map_.find(name) == attached_body_map_.end())
      scene_objects[name] = obj;

  // Minimum distance involving planned articulation
  for (const auto &[art_name, art] : planned_articulation_map_) {
    auto fcl_model = art->getFCLModel();
    auto col_objs = fcl_model->getCollisionObjects();
    auto col_link_names = fcl_model->getCollisionLinkNames();

    // Minimum distance to unplanned articulation
    for (const auto &art2 : unplanned_articulations) {
      auto art_name2 = art2->getName();
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs.size(); i++)
        for (size_t j = 0; j < col_objs2.size(); j++)
          if (auto type =
                  acm_->getAllowedCollision(col_link_names[i], col_link_names2[j]);
              !type || type == collision_detection::AllowedCollision::NEVER) {
            result.clear();
            ::fcl::distance(col_objs[i].get(), col_objs2[j].get(), request, result);
            if (result.min_distance < ret.min_distance) {
              ret.res = result;
              ret.min_distance = result.min_distance;
              ret.distance_type = "articulation_articulation";
              ret.object_name1 = art_name;
              ret.object_name2 = art_name2;
              ret.link_name1 = col_link_names[i];
              ret.link_name2 = col_link_names2[j];
            }
          }
    }

    // Minimum distance to scene objects
    for (const auto &[name, obj] : scene_objects)
      for (size_t i = 0; i < col_objs.size(); i++)
        if (auto type = acm_->getAllowedCollision(col_link_names[i], name);
            !type || type == collision_detection::AllowedCollision::NEVER) {
          result.clear();
          ::fcl::distance(col_objs[i].get(), obj.get(), request, result);
          if (result.min_distance < ret.min_distance) {
            ret.res = result;
            ret.min_distance = result.min_distance;
            ret.distance_type = "articulation_sceneobject";
            ret.object_name1 = art_name;
            ret.object_name2 = name;
            ret.link_name1 = col_link_names[i];
            ret.link_name2 = name;
          }
        }
  }

  // Minimum distance involving attached_body_map_
  for (const auto &[attached_body_name, attached_body] : attached_body_map_) {
    auto attached_obj = attached_body->getObject();

    // Minimum distance to unplanned articulation
    for (const auto &art2 : unplanned_articulations) {
      auto art_name2 = art2->getName();
      auto fcl_model2 = art2->getFCLModel();
      auto col_objs2 = fcl_model2->getCollisionObjects();
      auto col_link_names2 = fcl_model2->getCollisionLinkNames();

      for (size_t i = 0; i < col_objs2.size(); i++)
        if (auto type =
                acm_->getAllowedCollision(attached_body_name, col_link_names2[i]);
            !type || type == collision_detection::AllowedCollision::NEVER) {
          result.clear();
          ::fcl::distance(attached_obj.get(), col_objs2[i].get(), request, result);
          if (result.min_distance < ret.min_distance) {
            ret.res = result;
            ret.min_distance = result.min_distance;
            ret.distance_type = "attach_articulation";
            ret.object_name1 = attached_body_name;
            ret.object_name2 = art_name2;
            ret.link_name1 = attached_body_name;
            ret.link_name2 = col_link_names2[i];
          }
        }
    }

    // Minimum distance to scene objects
    for (const auto &[name, obj] : scene_objects)
      if (auto type = acm_->getAllowedCollision(attached_body_name, name);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        ::fcl::distance(attached_obj.get(), obj.get(), request, result);
        if (result.min_distance < ret.min_distance) {
          ret.res = result;
          ret.min_distance = result.min_distance;
          ret.distance_type = "attach_sceneobject";
          ret.object_name1 = attached_body_name;
          ret.object_name2 = name;
          ret.link_name1 = attached_body_name;
          ret.link_name2 = name;
        }
      }
  }
  return ret;
}

template <typename S>
WorldDistanceResultTpl<S> PlanningWorldTpl<S>::distanceFull(
    const DistanceRequest &request) const {
  auto ret1 = distanceSelf(request);
  auto ret2 = distanceOthers(request);
  return ret1.min_distance < ret2.min_distance ? ret1 : ret2;
}

}  // namespace mplib
