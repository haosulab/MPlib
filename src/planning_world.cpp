#include "mplib/planning_world.h"

#include <memory>

#include "mplib/macros_utils.h"
#include "mplib/urdf_utils.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_PLANNING_WORLD(S)     \
  template struct WorldCollisionResultTpl<S>; \
  template class PlanningWorldTpl<S>

DEFINE_TEMPLATE_PLANNING_WORLD(float);
DEFINE_TEMPLATE_PLANNING_WORLD(double);

template <typename S>
PlanningWorldTpl<S>::PlanningWorldTpl(
    const std::vector<ArticulatedModelPtr> &articulations,
    const std::vector<std::string> &articulation_names,
    const std::vector<CollisionObjectPtr> &normal_objects,
    const std::vector<std::string> &normal_object_names, int move_articulation_id)
    : articulations_(articulations),
      articulation_names_(articulation_names),
      move_articulation_id_(move_articulation_id),
      has_point_cloud_(false),
      has_attach_(false),
      use_point_cloud_(false),
      use_attach_(false) {
  ASSERT(articulations.size() == articulation_names.size(),
         "articulations and articulation_names should have the same size");
  ASSERT(normal_objects.size() == normal_object_names.size(),
         "normal_objects and normal_object_names should have the same size");
  for (size_t i = 0; i < normal_objects.size(); i++) {
    normal_object_map_[normal_object_names[i]] = normal_objects[i];
  }
}

template <typename S>
void PlanningWorldTpl<S>::setQpos(const int &index, const VectorX<S> &state) {
  articulations_[index]->setQpos(state);
}

template <typename S>
void PlanningWorldTpl<S>::updatePointCloud(const MatrixX3<S> &vertices,
                                           const double &radius) {
  std::shared_ptr<octomap::OcTree> p_octree = std::make_shared<octomap::OcTree>(radius);
  for (const auto &row : vertices.rowwise())
    p_octree->updateNode(octomap::point3d(row(0), row(1), row(2)), true);
  point_cloud_ = std::make_shared<CollisionObject>(
      std::make_shared<fcl::OcTree<S>>(p_octree), Transform3<S>::Identity());
  has_point_cloud_ = true;
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedTool(CollisionGeometryPtr p_geom, int link_id,
                                             const Vector7<S> &pose) {
  attach_link_id_ = link_id;
  // linear here means the upper left 3x3 matrix, which is not necessarily a rotation
  // matrix if scaling is involved
  attach_to_link_pose_.linear() =
      Quaternion<S>(pose[3], pose[4], pose[5], pose[6]).matrix();
  attach_to_link_pose_.translation() = pose.head(3);
  attached_tool_ = std::make_shared<CollisionObject>(p_geom, attach_to_link_pose_);
  has_attach_ = true;
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedSphere(S radius, int link_id,
                                               const Vector7<S> &pose) {
  CollisionGeometryPtr collision_geometry = std::make_shared<fcl::Sphere<S>>(radius);
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedBox(const Vector3<S> &size, int link_id,
                                            const Vector7<S> &pose) {
  CollisionGeometryPtr collision_geometry =
      std::make_shared<fcl::Box<S>>(size[0], size[1], size[2]);
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::updateAttachedMesh(const std::string &mesh_path, int link_id,
                                             const Vector7<S> &pose) {
  CollisionGeometryPtr collision_geometry =
      load_mesh_as_BVH(mesh_path, Vector3<S>(1, 1, 1));
  updateAttachedTool(collision_geometry, link_id, pose);
}

template <typename S>
void PlanningWorldTpl<S>::setQposAll(const VectorX<S> &state) {
  size_t total_dim = 0;
  for (size_t i = 0; i < articulations_.size(); i++) {
    auto n = articulations_[i]->getQposDim();
    auto segment =
        state.segment(total_dim, total_dim + n);  //[total_dim, total_dim + n)
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
    size_t index, const CollisionRequest &request) {
  std::vector<WorldCollisionResult> ret;
  auto fcl_model = articulations_[index]->getFCLModel();
  auto results = fcl_model.collideFull(request);
  auto CollisionLinkNames = fcl_model.getCollisionLinkNames();
  auto CollisionPairs = fcl_model.getCollisionPairs();
  for (size_t j = 0; j < results.size(); j++) {
    if (results[j].isCollision()) {
      WorldCollisionResult tmp;
      auto x = CollisionPairs[j].first, y = CollisionPairs[j].second;
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
    size_t index, const CollisionRequest &request) {
  std::vector<WorldCollisionResult> ret;
  auto pinocchio_model = articulations_[index]->getPinocchioModel();
  auto fcl_model = articulations_[index]->getFCLModel();
  auto CollisionObjects = fcl_model.getCollisionObjects();
  auto CollisionLinkNames = fcl_model.getCollisionLinkNames();

  // collision with other articulationed objects
  for (size_t i = 0; i < articulations_.size(); i++) {
    if (i == index) continue;  // don't collide with itself
    auto fcl_model_other = articulations_[i]->getFCLModel();
    auto CollisionObjects_other = fcl_model.getCollisionObjects();
    auto CollisionLinkNames_other = fcl_model.getCollisionLinkNames();
    for (size_t j = 0; j < CollisionObjects.size(); j++) {
      for (size_t k = 0; k < CollisionObjects_other.size(); k++) {
        CollisionResult result;
        result.clear();
        ::fcl::collide(CollisionObjects[j].get(), CollisionObjects_other[k].get(),
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
      ::fcl::collide(CollisionObjects[i].get(), normal_object.get(), request, result);
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
        ::fcl::collide(CollisionObjects[i].get(), point_cloud_.get(), request, result);
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

  if (use_attach_ &&
      use_point_cloud_) {  // TODO: attached box with other articulated objects
    if (has_attach_ == false) {
      print_warning("No Attached Box Provided but use_attach is true!");
    }
    if (has_point_cloud_ == false) {
      print_warning("No Point Cloud Provided!");
    } else {  // currently, only collide with the point cloud, only support one
              // articulation
      Vector7<S> link_pose = pinocchio_model.getLinkPose(attach_link_id_);
      Transform3<S> pose;
      pose.linear() =
          Quaternion<S>(link_pose[3], link_pose[4], link_pose[5], link_pose[6])
              .matrix();
      pose.translation() = link_pose.head(3);
      pose = pose * attach_to_link_pose_;
      // std::cout << attach_link_id << std::endl;
      // std::cout << "attached box pose: " << pose.linear() << std::endl;
      attached_tool_.get()->setTransform(pose);

      CollisionResult result;
      result.clear();
      ::fcl::collide(attached_tool_.get(), point_cloud_.get(), request, result);
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
    size_t index, const CollisionRequest &request) {
  std::vector<WorldCollisionResult> ret1 = selfCollide(index, request);
  std::vector<WorldCollisionResult> ret2 = collideWithOthers(index, request);
  ret1.insert(ret1.end(), ret2.begin(), ret2.end());
  return ret1;
}

template <typename S>
bool PlanningWorldTpl<S>::collide() {
  std::vector<WorldCollisionResult> ret = collideFull(0, CollisionRequest());
  return ret.size() > 0;
  /*for (size_t i = 0; i < articulations.size(); i++)
      if (articulation_flags[i])
          if (articulations[i]->getFCLModel().collide())
              return true;*/
  // return false;
}

/*template<typename S>
std::vector<WorldCollisionResultTpl<S>>
PlanningWorldTpl<S>::collideFull(void) { std::vector<WorldCollisionResult> ret;
    for (size_t i = 0; i < articulations.size(); i++)
        if (articulation_flags[i]) {
            auto fcl_model = articulations[i]->getFCLModel();
            auto results = fcl_model.collideFull();
            auto CollisionLinkNames = fcl_model.getCollisionLinkNames();
            auto CollisionPairs = fcl_model.getCollisionPairs();
            for (size_t j = 0; j < results.size(); j++) {
                WorldCollisionResult tmp;
                auto x = CollisionPairs[j].first, y = CollisionPairs[j].second;
                tmp.res = results[j];
                tmp.object_id1 = i;
                tmp.object_id2 = i;
                tmp.object_type1 = "articulation";
                tmp.object_type2 = "articulation";
                tmp.link_name1 = CollisionLinkNames[x];
                tmp.link_name2 = CollisionLinkNames[y];
                ret.push_back(tmp);
            }
        }
    return ret;
}*/

}  // namespace mplib
