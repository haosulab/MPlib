#pragma once

#include <memory>
#include <string>

#include <assimp/Importer.hpp>
#include <kdl/tree.hpp>
#include <urdf_model/pose.h>
#include <urdf_model/types.h>
#include <urdf_world/types.h>

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

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_URDF_UTILS(S)                                                \
  extern template Transform3<S> se3_to_transform<S>(const pinocchio::SE3<S> &T);      \
  extern template pinocchio::SE3<S> transform_to_se3<S>(const Transform3<S> &T);      \
  extern template Transform3<S> pose_to_transform<S>(const urdf::Pose &M);            \
  extern template pinocchio::SE3<S> pose_to_se3<S>(const urdf::Pose &M);              \
  extern template pinocchio::Inertia<S> convert_inertial<S>(const urdf::Inertial &Y); \
  extern template pinocchio::Inertia<S> convert_inertial<S>(                          \
      const urdf::InertialSharedPtr &Y);                                              \
  extern template std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<S>>> load_mesh_as_BVH<S>( \
      const std::string &mesh_path, const Vector3<S> &scale);                         \
  extern template std::shared_ptr<fcl::Convex<S>> load_mesh_as_Convex<S>(             \
      const std::string &mesh_path, const Vector3<S> &scale)

DECLARE_TEMPLATE_URDF_UTILS(float);
DECLARE_TEMPLATE_URDF_UTILS(double);

}  // namespace mplib
