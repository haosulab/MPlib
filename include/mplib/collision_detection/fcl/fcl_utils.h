#pragma once

#include <string>

#include "mplib/collision_detection/fcl/types.h"
#include "mplib/types.h"

namespace mplib::collision_detection::fcl {

template <typename S>
fcl::BVHModel_OBBRSSPtr<S> loadMeshAsBVH(const std::string &mesh_path,
                                         const Vector3<S> &scale);

template <typename S>
fcl::ConvexPtr<S> loadMeshAsConvex(const std::string &mesh_path,
                                   const Vector3<S> &scale);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_FCL_UTILS(S)                                                 \
  extern template fcl::BVHModel_OBBRSSPtr<S> loadMeshAsBVH<S>(                        \
      const std::string &mesh_path, const Vector3<S> &scale);                         \
  extern template fcl::ConvexPtr<S> loadMeshAsConvex<S>(const std::string &mesh_path, \
                                                        const Vector3<S> &scale)

DECLARE_TEMPLATE_FCL_UTILS(float);
DECLARE_TEMPLATE_FCL_UTILS(double);

}  // namespace mplib::collision_detection::fcl
