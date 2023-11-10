#include "fcl/geometry/octree/octree.h"
#include "macros_utils.hpp"
#include "planning_world.h"
#include "fcl_model.h"
#include "urdf_utils.h"


#define DEFINE_TEMPLATE_PW(DATATYPE) template class PlanningWorldTpl<DATATYPE>; template class WorldCollisionResultTpl<DATATYPE>;

DEFINE_TEMPLATE_PW(double)

DEFINE_TEMPLATE_PW(float)


template<typename DATATYPE>
PlanningWorldTpl<DATATYPE>::PlanningWorldTpl(std::vector<ArticulatedModel_ptr> const &articulations,
                                             std::vector<std::string> const & articulation_names,
                                             std::vector<CollisionObject_ptr> const &normal_objects, std::vector<std::string> const & normal_object_names,
                                             int move_articulation_id):
        articulations(articulations),
        normal_objects(normal_objects),
        articulation_names(articulation_names),
        normal_object_names(normal_object_names),
        move_articulation_id(move_articulation_id),
        has_point_cloud(false),
        use_point_cloud(false),
        has_attach(false),
        use_attach(false) {}

template<typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::setQpos(int const &index, VectorX const &state) {
    articulations[index]->setQpos(state);
}

template<typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updatePointCloud(Matrixx3 const& vertices, const double& radius) {
    std::shared_ptr<octomap::OcTree> p_octree = std::make_shared<octomap::OcTree>(radius);
    for (size_t i = 0; i < vertices.rows(); i++) {
        p_octree->updateNode(vertices(i, 0), vertices(i, 1), vertices(i, 2), true);
    }
    point_cloud = std::make_shared<CollisionObject>(
        std::make_shared< fcl::OcTree<DATATYPE> >(p_octree), Transform3::Identity());
    has_point_cloud = true;
}

template<typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedTool(CollisionGeometry_ptr p_geom, int link_id, Vector7 const &pose) {
    attach_link_id = link_id;
    // linear here means the upper left 3x3 matrix, which is not necessarily a rotation matrix if scaling is involved
    attach_to_link_pose.linear() = Quaternion(pose[3], pose[4], pose[5], pose[6]).matrix();
    attach_to_link_pose.translation() = pose.head(3);
    attached_tool = std::make_shared<CollisionObject>(p_geom, attach_to_link_pose);
    has_attach = true;
}

template<typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedSphere(DATATYPE radius, int link_id, const Vector7 &pose) {
    CollisionGeometry_ptr collision_geometry = std::make_shared<fcl::Sphere<DATATYPE>>(radius);
    updateAttachedTool(collision_geometry, link_id, pose);
}

template<typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedBox(Vector3 const &size, int link_id, Vector7 const &pose) {
    CollisionGeometry_ptr collision_geometry = std::make_shared<fcl::Box<DATATYPE>>(size[0], size[1], size[2]);
    updateAttachedTool(collision_geometry, link_id, pose);
}

template<typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::updateAttachedMesh(std::string const &mesh_path, int link_id, Vector7 const &pose) {
    CollisionGeometry_ptr collision_geometry = load_mesh_as_BVH(mesh_path, Vector3(1,1,1));
    updateAttachedTool(collision_geometry, link_id, pose);
}

template<typename DATATYPE>
void PlanningWorldTpl<DATATYPE>::setQposAll(VectorX const &state) {
    size_t total_dim = 0;
    for (size_t i = 0; i < articulations.size(); i++) {
        auto n = articulations[i]->getQposDim();
        auto segment = state.segment(total_dim, total_dim + n); //[total_dim, total_dim + n)
        ASSERT(segment.size() == n, "Bug with size " + std::to_string(segment.size()) + " " + std::to_string(n));
        setQpos(i, segment);
        total_dim += n;
    }
    ASSERT(total_dim == state.size(), "State dimension is not correct");
}

template<typename DATATYPE>
std::vector<WorldCollisionResultTpl<DATATYPE>> PlanningWorldTpl<DATATYPE>::selfCollide(int index, CollisionRequest const& request) {
    std::vector<WorldCollisionResult> ret;
    auto fcl_model = articulations[index]->getFCLModel();
    auto results = fcl_model.collideFull(request);
    auto CollisionLinkNames = fcl_model.getCollisionLinkNames();
    auto CollisionPairs = fcl_model.getCollisionPairs();
    for (size_t j = 0; j < results.size(); j++) 
        if (results[j].isCollision())
        {
            WorldCollisionResult tmp;
            auto x = CollisionPairs[j].first, y = CollisionPairs[j].second;
            tmp.res = results[j];
            tmp.object_name1 = articulation_names[index];
            tmp.object_name2 = articulation_names[index];
            tmp.collision_type = "self";
            tmp.link_name1 = CollisionLinkNames[x];
            tmp.link_name2 = CollisionLinkNames[y];
            ret.push_back(tmp);
        }
    return ret;
}

template<typename DATATYPE>
std::vector<WorldCollisionResultTpl<DATATYPE>> PlanningWorldTpl<DATATYPE>::collideWithOthers(int index, CollisionRequest const& request) {
    std::vector<WorldCollisionResult> ret;
    auto pinocchio_model = articulations[index]->getPinocchioModel();
    auto fcl_model = articulations[index]->getFCLModel();
    auto CollisionObjects = fcl_model.getCollisionObjects();
    auto CollisionLinkNames = fcl_model.getCollisionLinkNames();

    for (size_t i = 0; i < articulations.size(); i++) {
        if (i == index)
            continue;
        auto fcl_model1 = articulations[i]->getFCLModel();
        auto CollisionObjects1 = fcl_model.getCollisionObjects();
        auto CollisionLinkNames1 = fcl_model.getCollisionLinkNames();
        for (size_t j = 0; j < CollisionObjects.size(); j++)
            for (size_t k = 0; k < CollisionObjects1.size(); k++) {
                CollisionResult result;
                result.clear();
                fcl::collide(CollisionObjects[j].get(), CollisionObjects1[k].get(), request, result);
                if (result.isCollision()) {
                    WorldCollisionResult tmp;
                    tmp.res = result;
                    tmp.object_name1 = articulation_names[index];
                    tmp.object_name2 = articulation_names[i];
                    tmp.collision_type = "articulation";
                    tmp.link_name1 = CollisionLinkNames[j];
                    tmp.link_name2 = CollisionLinkNames[k];
                    ret.push_back(tmp);
                } 
            }
    }
    for (size_t i = 0; i < CollisionObjects.size(); i++)
        for (size_t j = 0; j < normal_objects.size(); j++) {
            CollisionResult result;
            result.clear();
            fcl::collide(CollisionObjects[i].get(), normal_objects[j].get(), request, result);
            if (result.isCollision()) {
                WorldCollisionResult tmp;
                tmp.res = result;
                tmp.object_name1 = articulation_names[index];
                tmp.object_name2 = normal_object_names[j];
                tmp.collision_type = "normal_object";
                tmp.link_name1 = CollisionLinkNames[i];
                tmp.link_name2 = normal_object_names[j];
                ret.push_back(tmp);
            } 
        }

    if (use_point_cloud) {
        if (has_point_cloud == false) {
            std::cout << "No Point Cloud Provided!" << std::endl; 
        }
        else {
            for (size_t i = 0; i < CollisionObjects.size(); i++) {
                CollisionResult result;
                result.clear();
                fcl::collide(CollisionObjects[i].get(), point_cloud.get(), request, result);
                if (result.isCollision()) {
                    WorldCollisionResult tmp;
                    tmp.res = result;
                    tmp.object_name1 = articulation_names[index];
                    tmp.object_name2 = "point_cloud";
                    tmp.collision_type = "point_cloud";
                    tmp.link_name1 = CollisionLinkNames[i];
                    tmp.link_name2 = "point_cloud";
                    ret.push_back(tmp);
                } 
            }
        }
    }
    if (use_attach && use_point_cloud) { // TODO: attached box with other articulated objects
        if (has_attach == false) {
            std::cout << "No Attached Box Provided!" << std::endl; 
        }
        else if (has_point_cloud == false) {
            std::cout << "No Point Cloud Provided!" << std::endl; 
        }
        else { // currently, only collide with the point cloud, only support one articulation
            Vector7 link_pose = pinocchio_model.getLinkPose(attach_link_id);
            Transform3 pose;
            pose.linear() = Quaternion(link_pose[3], link_pose[4], link_pose[5], link_pose[6]).matrix();
            pose.translation() = link_pose.head(3);
            pose = pose * attach_to_link_pose;
            //std::cout << "attached box pose: " << pose << std::endl;
            attached_tool.get()->setTransform(pose);

            CollisionResult result;
            result.clear();
            fcl::collide(attached_tool.get(), point_cloud.get(), request, result);
            if (result.isCollision()) {
                WorldCollisionResult tmp;
                tmp.res = result;
                tmp.object_name1 = "attached_box";
                tmp.object_name2 = "point_cloud";
                tmp.collision_type = "attach";
                tmp.link_name1 = "attached_box";
                tmp.link_name2 = "point_cloud";
                ret.push_back(tmp);
            } 
        }

    }
    return ret;
}

template<typename DATATYPE>
std::vector<WorldCollisionResultTpl<DATATYPE>> PlanningWorldTpl<DATATYPE>::collideFull(int index, CollisionRequest const& request) {
    std::vector<WorldCollisionResult> ret1 = selfCollide(index, request);
    std::vector<WorldCollisionResult> ret2 = collideWithOthers(index, request);
    ret1.insert(ret1.end(), ret2.begin(), ret2.end());
    return ret1;
}


template<typename DATATYPE>
bool PlanningWorldTpl<DATATYPE>::collide() {
    std::vector<WorldCollisionResult> ret = collideFull(0, CollisionRequest());
    return ret.size() > 0;
    /*for (size_t i = 0; i < articulations.size(); i++)
        if (articulation_flags[i])
            if (articulations[i]->getFCLModel().collide())
                return true;*/
    //return false;
}


/*template<typename DATATYPE>
std::vector<WorldCollisionResultTpl<DATATYPE>> PlanningWorldTpl<DATATYPE>::collideFull(void) {
    std::vector<WorldCollisionResult> ret;
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