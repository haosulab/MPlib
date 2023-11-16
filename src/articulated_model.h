#pragma once

#include "macros_utils.hpp"
#include "pinocchio_model.h"
#include "fcl_model.h"


template<typename DATATYPE>
class ArticulatedModelTpl {
private:
    DEFINE_TEMPLATE_EIGEN(DATATYPE);
    using PinocchioModel = PinocchioModelTpl<DATATYPE>;
    using FCLModel = FCLModelTpl<DATATYPE>;

    PinocchioModel pinocchio_model;
    FCLModel fcl_model;

    std::vector<std::string> user_link_names;
    std::vector<std::string> user_joint_names;  // all links and joints you want to control. order matters

    std::vector<size_t> move_group_user_joints;
    std::vector<std::string> move_group_end_effectors;
    VectorX current_qpos; // The planning world only update the state in planning group.

    int qpos_dim;
    bool verbose;

    // the base pose of the robot
    Vector7 base_pose;
    Transform3 base_tf;

public:
    ArticulatedModelTpl(std::string const &urdf_filename,
                        std::string const &srdf_filename,
                        Vector3 const &gravity,
                        std::vector<std::string> const &joint_names = {},
                        std::vector<std::string> const &link_names = {},
                        bool const &verbose=true,
                        bool const &convex=false);

    PinocchioModelTpl<DATATYPE>& getPinocchioModel() { return pinocchio_model; }

    FCLModelTpl<DATATYPE>& getFCLModel() { return fcl_model; }

    void setMoveGroup(std::string const& end_effector);

    void setMoveGroup(std::vector<std::string> const& end_effectors);

    std::vector<size_t> getMoveGroupJointIndices(void) { return move_group_user_joints; }

    std::vector<std::string> getMoveGroupJointName(void);

    std::vector<std::string> getUserJointNames(void) { return user_joint_names; }

    std::vector<std::string> getUserLinkNames(void) { return user_link_names; }

    std::vector<std::string> getMoveGroupEndEffectors(void) { return move_group_end_effectors; }

    size_t getQposDim(void) { return qpos_dim; }

    VectorX getQpos(void) { return current_qpos; }

    void setQpos(VectorX const& qpos, bool const& full=false);

    void updateSRDF(std::string const &srdf) { fcl_model.removeCollisionPairsFromSrdf(srdf); }

    void setBasePose(const Vector7 &pose);

    Vector7 getBasePose() { return base_pose; }
};

template<typename T>
using ArticulatedModelTpl_ptr = std::shared_ptr<ArticulatedModelTpl<T>>;

using ArticulatedModeld = ArticulatedModelTpl<double>;
using ArticulatedModelf = ArticulatedModelTpl<float>;
using ArticulatedModeld_ptr = ArticulatedModelTpl_ptr<double>;
using ArticulatedModelf_ptr = ArticulatedModelTpl_ptr<float>;
