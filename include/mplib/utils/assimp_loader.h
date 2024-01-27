#pragma once

#include <string>
#include <vector>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <fcl/math/triangle.h>

#include "mplib/types.h"

namespace mplib {

class AssimpLoader {
 public:
  AssimpLoader();
  ~AssimpLoader();

  void load(const std::string &resource_path);

  template <typename S>
  size_t dfsBuildMesh(const Vector3<S> &scale, int vertices_offset,
                      std::vector<Vector3<S>> &vertices,
                      std::vector<fcl::Triangle> &triangles) const {
    return _dfsBuildMesh(scene_->mRootNode, scale, vertices_offset, vertices,
                         triangles);
  }

 private:
  template <typename S>
  size_t _dfsBuildMesh(const aiNode *node, const Vector3<S> &scale, int vertices_offset,
                       std::vector<Vector3<S>> &vertices,
                       std::vector<fcl::Triangle> &triangles) const;

  Assimp::Importer *importer_;
  const aiScene *scene_;
};

// Explicit Template Instantiation Declaration =========================================
#define DECLARE_TEMPLATE_ASSIMP_LOADER(S)                               \
  extern template size_t AssimpLoader::_dfsBuildMesh<S>(                \
      const aiNode *node, const Vector3<S> &scale, int vertices_offset, \
      std::vector<Vector3<S>> &vertices, std::vector<fcl::Triangle> &triangles) const

DECLARE_TEMPLATE_ASSIMP_LOADER(float);
DECLARE_TEMPLATE_ASSIMP_LOADER(double);

}  // namespace mplib
