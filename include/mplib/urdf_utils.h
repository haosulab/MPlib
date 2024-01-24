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
Transform3<S> toTransform(const pinocchio::SE3<S> &T);

template <typename S>
Transform3<S> toTransform(const urdf::Pose &M);

template <typename S>
pinocchio::SE3<S> toSE3(const Transform3<S> &T);

template <typename S>
pinocchio::SE3<S> toSE3(const urdf::Pose &M);

template <typename S>
pinocchio::Inertia<S> convertInertial(const urdf::Inertial &Y);

template <typename S>
pinocchio::Inertia<S> convertInertial(const urdf::InertialSharedPtr &Y);

struct AssimpLoader {
  AssimpLoader();
  ~AssimpLoader();

  void load(const std::string &resource_path);

  Assimp::Importer *importer;
  const aiScene *scene;
};

template <typename S>
std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<S>>> loadMeshAsBVH(
    const std::string &mesh_path, const Vector3<S> &scale);

template <typename S>
std::shared_ptr<fcl::Convex<S>> loadMeshAsConvex(const std::string &mesh_path,
                                                 const Vector3<S> &scale);

bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr &robot_model,
                       KDL::Tree &tree, std::string &tree_root_name,
                       const bool &verbose = false);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_URDF_UTILS(S)                                               \
  extern template Transform3<S> toTransform<S>(const pinocchio::SE3<S> &T);          \
  extern template Transform3<S> toTransform<S>(const urdf::Pose &M);                 \
  extern template pinocchio::SE3<S> toSE3<S>(const Transform3<S> &T);                \
  extern template pinocchio::SE3<S> toSE3<S>(const urdf::Pose &M);                   \
  extern template pinocchio::Inertia<S> convertInertial<S>(const urdf::Inertial &Y); \
  extern template pinocchio::Inertia<S> convertInertial<S>(                          \
      const urdf::InertialSharedPtr &Y);                                             \
  extern template std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<S>>> loadMeshAsBVH<S>(   \
      const std::string &mesh_path, const Vector3<S> &scale);                        \
  extern template std::shared_ptr<fcl::Convex<S>> loadMeshAsConvex<S>(               \
      const std::string &mesh_path, const Vector3<S> &scale)

DECLARE_TEMPLATE_URDF_UTILS(float);
DECLARE_TEMPLATE_URDF_UTILS(double);

}  // namespace mplib
