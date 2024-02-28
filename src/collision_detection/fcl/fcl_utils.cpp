#include "mplib/collision_detection/fcl/fcl_utils.h"

#include <memory>

#include "mplib/utils/assimp_loader.h"

namespace mplib::collision_detection::fcl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_FCL_UTILS(S)                                                 \
  template fcl::BVHModel_OBBRSSPtr loadMeshAsBVH<S>(const std::string &mesh_path, \
                                                       const Vector3<S> &scale);     \
  template fcl::ConvexPtr loadMeshAsConvex<S>(const std::string &mesh_path,       \
                                                 const Vector3<S> &scale)

// DEFINE_TEMPLATE_FCL_UTILS(float);
DEFINE_TEMPLATE_FCL_UTILS(double);

template <typename S>
fcl::BVHModel_OBBRSSPtr loadMeshAsBVH(const std::string &mesh_path,
                                         const Vector3<S> &scale) {
  // TODO[Xinsong] change to a global loader so we do not initialize it every time
  auto loader = AssimpLoader();
  loader.load(mesh_path);

  std::vector<fcl::Vec3f> vertices;
  std::vector<fcl::Triangle> triangles;
  loader.dfsBuildMesh<S>(scale, 0, vertices, triangles);

  auto geom = std::make_shared<fcl::BVHModel_OBBRSS>();
  geom->beginModel();
  geom->addSubModel(vertices, triangles);
  geom->endModel();
  return geom;
}

template <typename S>
fcl::ConvexPtr loadMeshAsConvex(const std::string &mesh_path,
                                   const Vector3<S> &scale) {
  auto loader = AssimpLoader();
  loader.load(mesh_path);

  std::vector<fcl::Vec3f> vertices;
  std::vector<hpp::fcl::Triangle> triangles;
  loader.dfsBuildMesh<S>(scale, 0, vertices, triangles);

  // need to turn vertices into hpp::fcl::Vec3f *points_
  auto hppfcl_points_ptr = new hpp::fcl::Vec3f[vertices.size()];
  for (size_t i = 0; i < vertices.size(); i++) {
    hppfcl_points_ptr[i] = vertices[i];
  }
  // hpp::fcl::Triangle *polygons_
  auto hppfcl_polygons_ptr = new hpp::fcl::Triangle[triangles.size()];
  for (int i = 0; i < triangles.size(); i++) {
    hppfcl_polygons_ptr[i] = triangles[i];
  }
  return std::make_shared<hpp::fcl::Convex<hpp::fcl::Triangle>>(true,
    hppfcl_points_ptr, vertices.size(), hppfcl_polygons_ptr, triangles.size());
}

}  // namespace mplib::collision_detection::fcl
