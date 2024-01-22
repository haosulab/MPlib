#pragma once

#include <assimp/DefaultLogger.hpp>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <kdl/tree.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "color_printing.h"
#include "fcl/common/types.h"
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"
#include "pinocchio/multibody/joint/fwd.hpp"
#include "types.h"

namespace mplib {

template <typename S>
Transform3<S> se3_to_transform(const pinocchio::SE3<S> &T);

template <typename S>
pinocchio::SE3<S> transform_to_se3(const Transform3<S> &T);

template <typename S>
Transform3<S> pose_to_transform(const urdf::Pose &M);

template <typename S>
pinocchio::SE3<S> pose_to_se3(const urdf::Pose &M);

template <typename S>
pinocchio::Inertia<S> convert_inertial(const urdf::Inertial &Y);

template <typename S>
pinocchio::Inertia<S> convert_inertial(const urdf::InertialSharedPtr &Y);

template <typename S>
std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<S>>> load_mesh_as_BVH(
    const std::string &mesh_path, const Vector3<S> &scale);

template <typename S>
std::shared_ptr<fcl::Convex<S>> load_mesh_as_Convex(const std::string &mesh_path,
                                                    const Vector3<S> &scale);

struct AssimpLoader {
  AssimpLoader();
  ~AssimpLoader();

  void load(const std::string &resource_path);

  Assimp::Importer *importer;
  const aiScene *scene;
};

bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr &robot_model,
                       KDL::Tree &tree, std::string &tree_root_name,
                       const bool &verbose = false);

}  // namespace mplib
