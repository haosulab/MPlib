#include <memory>
#include <hpp/fcl/octree.h>

#include "mplib/planning_world.h"
#include "mplib/collision_detection/fcl/fcl_utils.h"
#include "mplib/macros/assert.h"
#include "mplib/utils/conversion.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_PLANNING_WORLD(S) template class PlanningWorldTpl<S>

// DEFINE_TEMPLATE_PLANNING_WORLD(float);
DEFINE_TEMPLATE_PLANNING_WORLD(double);

template <typename S>
PlanningWorldTpl<S>::PlanningWorldTpl(
    const std::vector<ArticulatedModelPtr> &articulations,
    const std::vector<std::string> &articulation_names,
    const std::vector<CollisionObjectPtr> &normal_objects,
    const std::vector<std::string> &normal_object_names, int move_articulation_id)
    : use_point_cloud_(false),
      use_attach_(false),
      articulations_(articulations),
      articulation_names_(articulation_names),
      move_articulation_id_(move_articulation_id),
      has_point_cloud_(false),
      has_attach_(false) {
  ASSERT(articulations.size() == articulation_names.size(),
         "articulations and articulation_names should have the same size");
  ASSERT(normal_objects.size() == normal_object_names.size(),
         "normal_objects and normal_object_names should have the same size");
  for (size_t i = 0; i < normal_objects.size(); i++) {
    normal_object_map_[normal_object_names[i]] = normal_objects[i];
  }
}

template <typename S>
void PlanningWorldTpl<S>::updatePointCloud(const MatrixX3<S> &vertices, double radius) {
  auto tree = std::make_shared<octomap::OcTree>(radius);
  for (const auto &row : vertices.rowwise())
    tree->updateNode(
        octomap::point3d {static_cast<float>(row(0)), static_cast<float>(row(1)),
                          static_cast<float>(row(2))},
        true);
  point_cloud_ = std::make_shared<CollisionObject>(
      std::make_shared<hpp::fcl::OcTree>(tree), hpp::fcl::Transform3f::Identity());
  has_point_cloud_ = true;
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedTool(const CollisionGeometryPtr &p_geom,
                                             int link_id, const Vector7<S> &pose) {
  attach_link_id_ = link_id;
  attach_to_link_pose_ = toIsometry<S>(pose);
  attached_tool_ = std::make_shared<CollisionObject>(p_geom,
      attach_to_link_pose_.linear(), attach_to_link_pose_.translation());
  has_attach_ = true;
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedBox(const Vector3<S> &size, int link_id,
                                            const Vector7<S> &pose) {
  const CollisionGeometryPtr collision_geometry =
      std::make_shared<hpp::fcl::Box>(size[0], size[1], size[2]);
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedSphere(S radius, int link_id,
                                               const Vector7<S> &pose) {
  const CollisionGeometryPtr collision_geometry =
      std::make_shared<hpp::fcl::Sphere>(radius);
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedMesh(const std::string &mesh_path, int link_id,
                                             const Vector7<S> &pose) {
  const CollisionGeometryPtr collision_geometry =
      collision_detection::fcl::loadMeshAsBVH(mesh_path, Vector3<S> {1, 1, 1});
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::setQpos(int index, const VectorX<S> &state) const {
  articulations_[index]->setQpos(state);
}

template <typename S>
void PlanningWorldTpl<S>::setQposAll(const VectorX<S> &state) const {
  size_t total_dim = 0;
  for (size_t i = 0; i < articulations_.size(); i++) {
    const auto n = articulations_[i]->getQposDim();
    const auto segment =
        state.segment(total_dim, total_dim + n);  // [total_dim, total_dim + n)
    ASSERT(static_cast<size_t>(segment.size()) == n,
           "Bug with size " + std::to_string(segment.size()) + " " + std::to_string(n));
    setQpos(i, segment);
    total_dim += n;
  }
  ASSERT(total_dim == static_cast<size_t>(state.size()),
         "State dimension is not correct");
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::selfCollide(
    size_t index, const CollisionRequest &request) const {
  std::vector<WorldCollisionResult> ret;
  const auto fcl_model = articulations_[index]->getFCLModel();
  const auto results = fcl_model->collideFull(request);
  const auto CollisionLinkNames = fcl_model->getCollisionLinkNames();
  const auto CollisionPairs = fcl_model->getCollisionPairs();
  for (size_t j = 0; j < results.size(); j++) {
    if (results[j].isCollision()) {
      WorldCollisionResult tmp;
      const auto x = CollisionPairs[j].first, y = CollisionPairs[j].second;
      tmp.res = results[j];
      tmp.object_name1 = articulation_names_[index];
      tmp.object_name2 = articulation_names_[index];
      tmp.collision_type = "self";
      tmp.link_name1 = CollisionLinkNames[x];
      tmp.link_name2 = CollisionLinkNames[y];
      ret.push_back(tmp);
    }
  }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::collideWithOthers(
    size_t index, const CollisionRequest &request) const {
  std::vector<WorldCollisionResult> ret;
  const auto pinocchio_model = articulations_[index]->getPinocchioModel();
  const auto fcl_model = articulations_[index]->getFCLModel();
  const auto CollisionObjects = fcl_model->getCollisionObjects();
  const auto CollisionLinkNames = fcl_model->getCollisionLinkNames();

  // collision with other articulationed objects
  for (size_t i = 0; i < articulations_.size(); i++) {
    if (i == index) continue;  // don't collide with itself
    const auto fcl_model_other = articulations_[i]->getFCLModel();
    const auto CollisionObjects_other = fcl_model_other->getCollisionObjects();
    const auto CollisionLinkNames_other = fcl_model_other->getCollisionLinkNames();
    for (size_t j = 0; j < CollisionObjects.size(); j++) {
      for (size_t k = 0; k < CollisionObjects_other.size(); k++) {
        CollisionResult result;
        result.clear();
        hpp::fcl::collide(CollisionObjects[j].get(), CollisionObjects_other[k].get(),
                          request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.object_name1 = articulation_names_[index];
          tmp.object_name2 = articulation_names_[i];
          tmp.collision_type = "articulation";
          tmp.link_name1 = CollisionLinkNames[j];
          tmp.link_name2 = CollisionLinkNames_other[k];
          ret.push_back(tmp);
        }
      }
    }
  }

  // collision with non-articulated objects
  for (size_t i = 0; i < CollisionObjects.size(); i++) {
    for (const auto &itm : normal_object_map_) {
      CollisionResult result;
      result.clear();
      const std::string &normal_object_name = itm.first;
      const auto &normal_object = itm.second;
      hpp::fcl::collide(CollisionObjects[i].get(), normal_object.get(), request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.object_name1 = articulation_names_[index];
        tmp.object_name2 = normal_object_name;
        tmp.collision_type = "normal_object";
        tmp.link_name1 = CollisionLinkNames[i];
        tmp.link_name2 = normal_object_name;
        ret.push_back(tmp);
      }
    }
  }

  if (use_point_cloud_) {
    if (has_point_cloud_ == false) {
      print_warning("No Point Cloud Provided!");
    } else {
      for (size_t i = 0; i < CollisionObjects.size(); i++) {
        CollisionResult result;
        result.clear();
        hpp::fcl::collide(CollisionObjects[i].get(), point_cloud_.get(), request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.object_name1 = articulation_names_[index];
          tmp.object_name2 = "point_cloud";
          tmp.collision_type = "point_cloud";
          tmp.link_name1 = CollisionLinkNames[i];
          tmp.link_name2 = "point_cloud";
          ret.push_back(tmp);
        }
      }
    }
  }

  if (use_attach_ && use_point_cloud_) {
    if (!has_attach_ || !has_point_cloud_) {
      if (!has_attach_)
        print_warning("No Attached Box Provided but use_attach is true!");
      if (!has_point_cloud_)
        print_warning("No Point Cloud Provided but use_point_cloud is true!");
    } else {  // currently, only collide with the point cloud, only support one
              // articulation
      const Vector7<S> link_pose = pinocchio_model->getLinkPose(attach_link_id_);
      auto link_pose_isometry = toIsometry<S>(link_pose) * attach_to_link_pose_;
      attached_tool_.get()->setTransform(link_pose_isometry.linear(),
                                         link_pose_isometry.translation());

      CollisionResult result;
      result.clear();
      hpp::fcl::collide(attached_tool_.get(), point_cloud_.get(), request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.object_name1 = "attached_tool";
        tmp.object_name2 = "point_cloud";
        tmp.collision_type = "attach";
        tmp.link_name1 = "attached_tool";
        tmp.link_name2 = "point_cloud";
        ret.push_back(tmp);
      }
    }
  }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> PlanningWorldTpl<S>::collideFull(
    size_t index, const CollisionRequest &request) const {
  std::vector<WorldCollisionResult> ret1 = selfCollide(index, request);
  const std::vector<WorldCollisionResult> ret2 = collideWithOthers(index, request);
  ret1.insert(ret1.end(), ret2.begin(), ret2.end());
  return ret1;
}

}  // namespace mplib
