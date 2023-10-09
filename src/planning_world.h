#pragma once

#include <vector>
#include "articulated_model.h"
#include "fcl_model.h"
#include "macros_utils.hpp"


template<typename DATATYPE>
struct WorldCollisionResultTpl {
    fcl::CollisionResult<DATATYPE> res;
    //size_t object_id1, object_id2;
    std::string collision_type, object_name1, object_name2, link_name1, link_name2;
};


template<typename T>
using WorldCollisionResultTpl_ptr = std::shared_ptr<WorldCollisionResultTpl<T>>;

using WorldCollisionResultd = WorldCollisionResultTpl<double>;
using WorldCollisionResultf = WorldCollisionResultTpl<float>;
using WorldCollisionResultd_ptr = WorldCollisionResultTpl_ptr<double>;
using WorldCollisionResultf_ptr = WorldCollisionResultTpl_ptr<float>;

template<typename DATATYPE>
class PlanningWorldTpl {
private:
    DEFINE_TEMPLATE_EIGEN(DATATYPE);
    using CollisionRequest = fcl::CollisionRequest<DATATYPE>;
    using CollisionResult = fcl::CollisionResult<DATATYPE>;

    using CollisionObject = fcl::CollisionObject<DATATYPE>;
    using CollisionObject_ptr = std::shared_ptr<CollisionObject>;

    using DynamicAABBTreeCollisionManager = fcl::DynamicAABBTreeCollisionManager<DATATYPE>;
    using BroadPhaseCollisionManager_ptr = std::shared_ptr<fcl::BroadPhaseCollisionManager<DATATYPE>>;

    using ArticulatedModel = ArticulatedModelTpl<DATATYPE>;
    using ArticulatedModel_ptr = ArticulatedModelTpl_ptr<DATATYPE>;

    using WorldCollisionResult = WorldCollisionResultTpl<DATATYPE>;
    using WorldCollisionResult_ptr = WorldCollisionResultTpl_ptr<DATATYPE>;

    std::vector<ArticulatedModel_ptr> articulations;
    //std::vector<bool> articulation_flags;
    std::vector<CollisionObject_ptr> normal_objects; // without articulation
    std::vector<std::string> articulation_names;
    std::vector<std::string> normal_object_names;
    int move_articulation_id, attach_link_id;
    CollisionObject_ptr point_cloud, attached_box;
    bool has_point_cloud, use_point_cloud, has_attach, use_attach;
    Transform3 attach_to_link_pose;
    //BroadPhaseCollisionManager_ptr normal_manager;


public:

    PlanningWorldTpl(std::vector<ArticulatedModel_ptr> const &articulations, std::vector<std::string> const & articulation_names,
                     std::vector<CollisionObject_ptr> const &normal_objects, std::vector<std::string> const & normal_object_names,
                     int plan_articulation_id = 0);
                     //std::vector<bool> const &articulation_flags);

    std::vector<ArticulatedModel_ptr> &getArticulations(void) {return articulations;}

    std::vector<CollisionObject_ptr> &getNormalObjects(void) {return normal_objects;}

    std::vector<std::string> &getArticulationNames() {return articulation_names;}

    std::vector<std::string> &getNormalObjectNames() {return normal_object_names;}

    void setMoveArticulationId(int id) {move_articulation_id = id;}

    int getMoveArticulationId() {return move_articulation_id;}

    void setUsePointCloud(bool const & use) {use_point_cloud = use;}

    void updatePointCloud(Matrixx3 const& vertices, double  const& resolution);

    void setUseAttach(bool const & use) {use_attach = use;}

    /**
     * @brief updateAttachedSphere
     * @param obj collision object
     * @param link_id id of the link to which the object is attached
     * @param pose the pose of the attached object w.r.t. the link it's attached to
     */
    void updateAttachedSphere(DATATYPE radius, int link_id, const Vector7 &pose);

    void updateAttachedBox(Vector3 const & size, int const & link_id, Vector7 const & pose);

    void printAttachedBoxPose() {
        auto tmp1 = attached_box.get()->getTranslation();
        auto tmp2 = attached_box.get()->getRotation();
        std::cout << tmp1 << ' ' << tmp2 << std::endl;
    }

    //std::vector<bool> &getArticulationFlags(void) { return articulation_flags; }

    void addArticulation(ArticulatedModel_ptr const &model, std::string const &name) { //bool const &planning = true) {
        articulations.push_back(model);
        articulation_names.push_back(name);
        //articulation_flags.push_back(planning);
    }

    void addArticulations(std::vector<ArticulatedModel_ptr> const &models, std::vector<std::string> const &names) {//std::vector<bool> const &planning) {
        articulations.insert(articulations.end(), models.begin(), models.end());
        articulation_names.insert(articulation_names.end(), names.begin(), names.end());
        //articulation_flags.insert(articulation_flags.end(), planning.begin(), planning.end());
    }

    void addNormalObject(CollisionObject_ptr const &collision_object, std::string const & name) {
        normal_objects.push_back(collision_object);
        normal_object_names.push_back(name);
    }

    void addNormalObjects(std::vector<CollisionObject_ptr> const &collision_objects, std::vector<std::string> const &names) {
        normal_objects.insert(normal_objects.end(), collision_objects.begin(), collision_objects.end());
        normal_object_names.insert(normal_object_names.end(), names.begin(), names.end());
    }

    void setQpos(int const &index, VectorX const &qpos);

    void setQposAll(VectorX const &qpos);

    //   bool collide_among_normal_objects()=0;

    bool collide();

    //std::vector<WorldCollisionResult> collideFull(void);
    std::vector<WorldCollisionResult> selfCollide(int index, CollisionRequest const& request=CollisionRequest());
    std::vector<WorldCollisionResult> collideWithOthers(int index, CollisionRequest const& request=CollisionRequest());
    std::vector<WorldCollisionResult> collideFull(int index, CollisionRequest const& request=CollisionRequest());
};

template<typename T>
using PlanningWorldTpl_ptr = std::shared_ptr<PlanningWorldTpl<T>>;

using PlanningWorldd = PlanningWorldTpl<double>;
using PlanningWorldf = PlanningWorldTpl<float>;
using PlanningWorldd_ptr = PlanningWorldTpl_ptr<double>;
using PlanningWorldf_ptr = PlanningWorldTpl_ptr<float>;

