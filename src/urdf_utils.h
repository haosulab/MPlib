#pragma once

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/multibody/joint/fwd.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "fcl/common/types.h"
#include "fcl/math/triangle.h"
#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/collision_request.h"

#include <assimp/DefaultLogger.hpp>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <kdl/tree.hpp>

#include "color_printing.h"

template<typename DATATYPE>
Eigen::Transform<DATATYPE, 3, Eigen::Isometry> se3_to_transform(const pinocchio::SE3Tpl<DATATYPE, 0> &T);

template<typename DATATYPE>
pinocchio::SE3Tpl<DATATYPE, 0> transform_to_se3(const Eigen::Transform<DATATYPE, 3, Eigen::Isometry>& T);

template<typename DATATYPE>
Eigen::Transform<DATATYPE, 3, Eigen::Isometry> pose_to_transform(const urdf::Pose &M);

template<typename DATATYPE>
pinocchio::SE3Tpl<DATATYPE, 0> pose_to_se3(const urdf::Pose &M);

template<typename DATATYPE>
pinocchio::InertiaTpl<DATATYPE, 0> convert_inertial(const urdf::Inertial &Y);

template<typename DATATYPE>
pinocchio::InertiaTpl<DATATYPE, 0> convert_inertial(const urdf::InertialSharedPtr &Y);

template<typename DATATYPE>
std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<DATATYPE>>>
load_mesh_as_BVH(const std::string &mesh_path, const Eigen::Matrix<DATATYPE, 3, 1> &scale);

template<typename DATATYPE>
std::shared_ptr<fcl::Convex<DATATYPE>>
load_mesh_as_Convex(const std::string &mesh_path, const Eigen::Matrix<DATATYPE, 3, 1> &scale);


struct AssimpLoader {
    AssimpLoader();
    ~AssimpLoader();

    void load(const std::string &resource_path);

    Assimp::Importer *importer;
    aiScene const *scene;
};


bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr &robot_model, KDL::Tree &tree, std::string &tree_root_name, bool const &verbose=false);

