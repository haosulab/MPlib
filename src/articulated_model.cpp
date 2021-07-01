#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/urdf/model.hxx"
#include "pinocchio/multibody/joint/fwd.hpp"

#include "pinocchio/parsers/utils.hpp"
#include "pinocchio/parsers/urdf/geometry.hxx"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>

#include <cmath>
#include <cstdlib>
#include <algorithm>

#include "articulated_model.h"
#include "urdf_utils.h"


#define DEFINE_TEMPLATE_AM(DATATYPE) template class ArticulatedModelTpl<DATATYPE>;

DEFINE_TEMPLATE_AM(float)

DEFINE_TEMPLATE_AM(double)


template<typename DATATYPE>
ArticulatedModelTpl<DATATYPE>::
ArticulatedModelTpl(std::string const &urdf_filename, std::string const &srdf_filename, Vector3 const &gravity,
                    std::vector<std::string> const &joint_names, std::vector<std::string> const &link_names,
                    bool const &verbose, bool const& convex) :verbose(verbose),
                                          pinocchio_model(urdf_filename, gravity, verbose),
                                          fcl_model(urdf_filename, verbose, convex) {
    //std::cout << "Verbose" << verbose << std::endl;
    user_link_names = link_names;
    user_joint_names = joint_names;
    pinocchio_model.setLinkOrder(link_names);
    pinocchio_model.setJointOrder(joint_names);
    fcl_model.setLinkOrder(link_names);
    fcl_model.removeCollisionPairsFromSrdf(srdf_filename);
    current_qpos = VectorX::Constant(pinocchio_model.getModel().nv, 0);
    setMoveGroup(user_link_names);
}

template<typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setMoveGroup(std::string const &end_effector) {
    std::vector<std::string> end_effectors = {end_effector};
    setMoveGroup(end_effectors);
}


template<typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setMoveGroup(std::vector<std::string> const &end_effectors) {
    move_group_end_effectors = end_effectors;
    move_group_user_joints = {};
    for (auto end_effector: end_effectors) {
        auto joint_i = pinocchio_model.getChainJointIndex(end_effector);
        move_group_user_joints.insert(move_group_user_joints.begin(), joint_i.begin(), joint_i.end());
    }
    std::sort(move_group_user_joints.begin(), move_group_user_joints.end());
    auto end_unique = std::unique(move_group_user_joints.begin(), move_group_user_joints.end());
    move_group_user_joints.erase(end_unique, move_group_user_joints.end());
    qpos_dim = 0;
    for (auto i: move_group_user_joints)
        qpos_dim += pinocchio_model.getJointDim(i);
}

template<typename DATATYPE>
std::vector<std::string> ArticulatedModelTpl<DATATYPE>::getMoveGroupJointName(void) {
    std::vector<std::string> ret;
    for (auto i: move_group_user_joints)
        ret.push_back(user_joint_names[i]);
    return ret;
}

template<typename DATATYPE>
void ArticulatedModelTpl<DATATYPE>::setQpos(VectorX const &qpos, bool const& full) {
    if (full)
        current_qpos = qpos;
    else {
        ASSERT(qpos.size() == qpos_dim,
               "Length is not correct, Dim of Q: " + std::to_string(qpos_dim) + " ,Len of qpos: " +
               std::to_string(qpos.size()));
        size_t len = 0;
        for (auto i: move_group_user_joints) {
            auto start_idx = pinocchio_model.getJointId(i), dim_i = pinocchio_model.getJointDim(i);
            for (size_t j = 0; j < dim_i; j++)
                current_qpos[start_idx + j] = qpos[len++];
        }
    }
    pinocchio_model.computeForwardKinematics(current_qpos);
    //std::cout << "current_qpos " << current_qpos << std::endl;
    std::vector<Transform3> link_pose;
    for (size_t i = 0; i < user_link_names.size(); i++) {
        Vector7 pose_i = pinocchio_model.getLinkPose(i);
        Transform3 tmp_i;
        tmp_i.linear() = Quaternion(pose_i[3], pose_i[4], pose_i[5], pose_i[6]).matrix();
        tmp_i.translation() = pose_i.head(3);
        //std::cout << pose_i << std::endl;
        link_pose.push_back(tmp_i);
    }
    fcl_model.updateCollisionObjects(link_pose);
}


