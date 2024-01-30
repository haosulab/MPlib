#include "mplib/collision_detection/fcl/fcl_utils.h"

#include <memory>

#include "mplib/utils/assimp_loader.h"

namespace mplib::collision_detection::fcl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_FCL_UTILS(S)                                                 \
  template fcl::BVHModel_OBBRSSPtr<S> loadMeshAsBVH<S>(const std::string &mesh_path, \
                                                       const Vector3<S> &scale);     \
  template fcl::ConvexPtr<S> loadMeshAsConvex<S>(const std::string &mesh_path,       \
                                                 const Vector3<S> &scale)

DEFINE_TEMPLATE_FCL_UTILS(float);
DEFINE_TEMPLATE_FCL_UTILS(double);

template <typename S>
fcl::BVHModel_OBBRSSPtr<S> loadMeshAsBVH(const std::string &mesh_path,
                                         const Vector3<S> &scale) {
  // TODO[Xinsong] change to a global loader so we do not initialize it every time
  auto loader = AssimpLoader();
  loader.load(mesh_path);

  std::vector<Vector3<S>> vertices;
  std::vector<fcl::Triangle> triangles;
  loader.dfsBuildMesh<S>(scale, 0, vertices, triangles);

  auto geom = std::make_shared<fcl::BVHModel_OBBRSS<S>>();
  geom->beginModel();
  geom->addSubModel(vertices, triangles);
  geom->endModel();
  return geom;
}

template <typename S>
fcl::ConvexPtr<S> loadMeshAsConvex(const std::string &mesh_path,
                                   const Vector3<S> &scale) {
  auto loader = AssimpLoader();
  loader.load(mesh_path);

  std::vector<Vector3<S>> vertices;
  std::vector<fcl::Triangle> triangles;
  loader.dfsBuildMesh<S>(scale, 0, vertices, triangles);

  auto faces = std::make_shared<std::vector<int>>();
  for (const auto &triangle : triangles) {
    faces->push_back(3);
    faces->push_back(triangle[0]);
    faces->push_back(triangle[1]);
    faces->push_back(triangle[2]);
  }
  const auto vertices_ptr = std::make_shared<const std::vector<Vector3<S>>>(vertices);
  return std::make_shared<fcl::Convex<S>>(vertices_ptr, triangles.size(), faces, true);
}

}  // namespace mplib::collision_detection::fcl
