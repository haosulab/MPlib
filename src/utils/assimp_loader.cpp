#include "mplib/utils/assimp_loader.h"

#include <sstream>
#include <stdexcept>

#include <assimp/postprocess.h>

#include "mplib/utils/color_printing.h"

namespace mplib {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_ASSIMP_LOADER(S)                                \
  template size_t AssimpLoader::_dfsBuildMesh<S>(                       \
      const aiNode *node, const Vector3<S> &scale, int vertices_offset, \
      std::vector<Vector3<S>> &vertices, std::vector<fcl::Triangle> &triangles) const

DEFINE_TEMPLATE_ASSIMP_LOADER(float);
DEFINE_TEMPLATE_ASSIMP_LOADER(double);

AssimpLoader::AssimpLoader() : importer_(new Assimp::Importer()) {
  // set list of ignored parameters (parameters used for rendering)
  importer_->SetPropertyInteger(
      AI_CONFIG_PP_RVC_FLAGS, aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
                                  aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
                                  aiComponent_LIGHTS | aiComponent_CAMERAS |
                                  aiComponent_TEXTURES | aiComponent_TEXCOORDS |
                                  aiComponent_MATERIALS | aiComponent_NORMALS);
}

AssimpLoader::~AssimpLoader() {
  if (importer_) delete importer_;
}

void AssimpLoader::load(const std::string &file_name) {
  importer_->SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
  scene_ = importer_->ReadFile(
      file_name.c_str(),
      aiProcess_SortByPType | aiProcess_Triangulate | aiProcess_RemoveComponent |
          aiProcess_ImproveCacheLocality |
          // TODO: I (Joseph Mirabel) have no idea whether degenerated triangles are
          // properly handled. Enabling aiProcess_FindDegenerates would throw an
          // exception when that happens. Is it too conservative ?
          // aiProcess_FindDegenerates |
          aiProcess_PreTransformVertices | aiProcess_JoinIdenticalVertices);

  if (!scene_)
    throw std::invalid_argument("Could not load resource " + file_name + "\n" +
                                importer_->GetErrorString() + "\n" +
                                "Hint: the mesh directory may be wrong.");

  if (!scene_->HasMeshes())
    throw std::invalid_argument("No meshes found in file " + file_name);

  // cast away the const since we need to give it a name
  const_cast<aiString &>(scene_->mName) = file_name;
}

template <typename S>
size_t AssimpLoader::_dfsBuildMesh(const aiNode *node, const Vector3<S> &scale,
                                   int vertices_offset,
                                   std::vector<Vector3<S>> &vertices,
                                   std::vector<fcl::Triangle> &triangles) const {
  if (!node) return 0;

  aiMatrix4x4 transform = node->mTransformation;
  const aiNode *pnode = node->mParent;
  while (pnode) {
    // Don't convert to y-up orientation, which is what the root node in Assimp does
    if (pnode->mParent != NULL) transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  size_t nbVertices = 0;
  for (size_t i = 0; i < node->mNumMeshes; i++) {
    const aiMesh *const input_mesh = scene_->mMeshes[node->mMeshes[i]];

    // Add the vertices
    S max_dim = 0;
    for (size_t j = 0; j < input_mesh->mNumVertices; j++) {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      vertices.push_back(Vector3<S> {static_cast<S>(p.x * scale[0]),
                                     static_cast<S>(p.y * scale[1]),
                                     static_cast<S>(p.z * scale[2])});
      max_dim = std::max({max_dim, std::abs(p.x) * scale[0], std::abs(p.y) * scale[1],
                          std::abs(p.z) * scale[2]});
    }
    if (max_dim < 1e-2 || max_dim > 1e1)
      print_warning("Mesh ", scene_->mName.C_Str(), " has side length ", max_dim,
                    "m which is suspiciously large or small. If this is indeed a unit "
                    "or scaling error, you can set the scale factor in the urdf file.");

    // add the indices
    for (size_t j = 0; j < input_mesh->mNumFaces; j++) {
      const aiFace &face = input_mesh->mFaces[j];
      if (face.mNumIndices != 3) {
        std::stringstream ss;
        ss << "Mesh " << input_mesh->mName.C_Str() << " has a face with "
           << face.mNumIndices << " vertices. This is not supported\n";
        ss << "Node name is: " << node->mName.C_Str() << "\n";
        ss << "Mesh index: " << i << "\n";
        ss << "Face index: " << j << "\n";
        throw std::invalid_argument(ss.str());
      }
      triangles.push_back(fcl::Triangle(vertices_offset + face.mIndices[0],
                                        vertices_offset + face.mIndices[1],
                                        vertices_offset + face.mIndices[2]));
    }

    nbVertices += input_mesh->mNumVertices;
  }

  for (size_t i = 0; i < node->mNumChildren; ++i)
    nbVertices +=
        _dfsBuildMesh(node->mChildren[i], scale, nbVertices, vertices, triangles);
  return nbVertices;
}

}  // namespace mplib
