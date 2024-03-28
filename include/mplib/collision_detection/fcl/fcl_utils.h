#pragma once

#include <string>

#include <fcl/narrowphase/collision.h>

#include "mplib/collision_detection/fcl/types.h"
#include "mplib/types.h"

namespace mplib::collision_detection::fcl {

/**
 * Load a triangle mesh from mesh_path as a non-convex collision object.
 *
 * @param mesh_path: path to the mesh
 * @param scale: mesh scale factor
 * @return: a shared_ptr to an fcl::BVHModel_OBBRSS<S> collision object
 */
template <typename S>
fcl::BVHModel_OBBRSSPtr<S> loadMeshAsBVH(const std::string &mesh_path,
                                         const Vector3<S> &scale);

/**
 * Load a convex mesh from mesh_path.
 *
 * @param mesh_path: path to the mesh
 * @param scale: mesh scale factor
 * @return: a shared_ptr to an fcl::Convex<S> collision object
 * @throws std::runtime_error if the mesh is not convex.
 */
template <typename S>
fcl::ConvexPtr<S> loadMeshAsConvex(const std::string &mesh_path,
                                   const Vector3<S> &scale);

template <typename S>
void collideFCLObjects(const fcl::FCLObject<S> &o1, const fcl::FCLObject<S> &o2,
                       const fcl::CollisionRequest<S> &request,
                       fcl::CollisionResult<S> &result);

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_FCL_UTILS(S)                                                 \
  extern template fcl::BVHModel_OBBRSSPtr<S> loadMeshAsBVH<S>(                        \
      const std::string &mesh_path, const Vector3<S> &scale);                         \
  extern template fcl::ConvexPtr<S> loadMeshAsConvex<S>(const std::string &mesh_path, \
                                                        const Vector3<S> &scale);     \
  extern template void collideFCLObjects(const fcl::FCLObject<S> &o1,                 \
                                         const fcl::FCLObject<S> &o2,                 \
                                         const fcl::CollisionRequest<S> &request,     \
                                         fcl::CollisionResult<S> &result)

DECLARE_TEMPLATE_FCL_UTILS(float);
DECLARE_TEMPLATE_FCL_UTILS(double);

}  // namespace mplib::collision_detection::fcl
