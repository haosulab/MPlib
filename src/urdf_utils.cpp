#include "urdf_utils.h"

#include <kdl/frames_io.hpp>

#define DEFINE_TEMPLATE_URDF_UTILS(DATATYPE)                                           \
  template pinocchio::InertiaTpl<DATATYPE, 0> convert_inertial<DATATYPE>(              \
      const urdf::Inertial &Y);                                                        \
  template pinocchio::InertiaTpl<DATATYPE, 0> convert_inertial<DATATYPE>(              \
      const urdf::InertialSharedPtr &Y);                                               \
  template pinocchio::SE3Tpl<DATATYPE, 0> pose_to_se3<DATATYPE>(const urdf::Pose &M);  \
  template Eigen::Transform<DATATYPE, 3, Eigen::Isometry> se3_to_transform<DATATYPE>(  \
      const pinocchio::SE3Tpl<DATATYPE, 0> &T);                                        \
  template Eigen::Transform<DATATYPE, 3, Eigen::Isometry> pose_to_transform<DATATYPE>( \
      const urdf::Pose &M);                                                            \
  template pinocchio::SE3Tpl<DATATYPE, 0> transform_to_se3<DATATYPE>(                  \
      const Eigen::Transform<DATATYPE, 3, Eigen::Isometry> &T);                        \
  template std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<DATATYPE>>>                       \
  load_mesh_as_BVH<DATATYPE>(const std::string &mesh_path,                             \
                             const Eigen::Matrix<DATATYPE, 3, 1> &scale);              \
  template std::shared_ptr<fcl::Convex<DATATYPE>> load_mesh_as_Convex<DATATYPE>(       \
      const std::string &mesh_path, const Eigen::Matrix<DATATYPE, 3, 1> &scale);

DEFINE_TEMPLATE_URDF_UTILS(float)

DEFINE_TEMPLATE_URDF_UTILS(double)

// copy code from pinocchio/src/parsers/urdf/model.cpp and add template to support float

template <typename DATATYPE>
Eigen::Transform<DATATYPE, 3, Eigen::Isometry> se3_to_transform(
    const pinocchio::SE3Tpl<DATATYPE, 0> &T) {
  Eigen::Transform<DATATYPE, 3, Eigen::Isometry> ret;
  ret.linear() = T.rotation_impl();
  ret.translation() = T.translation_impl();
  return ret;
}

template <typename DATATYPE>
pinocchio::SE3Tpl<DATATYPE, 0> transform_to_se3(
    const Eigen::Transform<DATATYPE, 3, Eigen::Isometry> &T) {
  return pinocchio::SE3Tpl<DATATYPE>(T.linear(), T.translation());
}

template <typename DATATYPE>
Eigen::Transform<DATATYPE, 3, Eigen::Isometry> pose_to_transform(const urdf::Pose &M) {
  const urdf::Vector3 &p = M.position;
  const urdf::Rotation &q = M.rotation;
  Eigen::Transform<DATATYPE, 3, Eigen::Isometry> ret =
      Eigen::Transform<DATATYPE, 3, Eigen::Isometry>::Identity();
  ret.linear() = Eigen::Quaternion<DATATYPE>(q.w, q.x, q.y, q.z).matrix();
  ret.translation() = Eigen::Matrix<DATATYPE, 3, 1>(p.x, p.y, p.z);
  return ret;
}

template <typename DATATYPE>
pinocchio::SE3Tpl<DATATYPE, 0> pose_to_se3(const urdf::Pose &M) {
  const urdf::Vector3 &p = M.position;
  const urdf::Rotation &q = M.rotation;
  return pinocchio::SE3Tpl<DATATYPE>(
      Eigen::Quaternion<DATATYPE>(q.w, q.x, q.y, q.z).matrix(),
      Eigen::Matrix<DATATYPE, 3, 1>(p.x, p.y, p.z));
}

template <typename DATATYPE>
pinocchio::InertiaTpl<DATATYPE, 0> convert_inertial(const urdf::Inertial &Y) {
  const urdf::Vector3 &p = Y.origin.position;
  const urdf::Rotation &q = Y.origin.rotation;
  const Eigen::Matrix<DATATYPE, 3, 1> com(p.x, p.y, p.z);
  const Eigen::Matrix<DATATYPE, 3, 3> &R =
      Eigen::Quaternion<DATATYPE>(q.w, q.x, q.y, q.z).matrix();
  Eigen::Matrix<DATATYPE, 3, 3> I;
  I << Y.ixx, Y.ixy, Y.ixz, Y.ixy, Y.iyy, Y.iyz, Y.ixz, Y.iyz, Y.izz;
  return pinocchio::InertiaTpl<DATATYPE, 0>(Y.mass, com, R * I * R.transpose());
}

template <typename DATATYPE>
pinocchio::InertiaTpl<DATATYPE, 0> convert_inertial(const urdf::InertialSharedPtr &Y) {
  if (Y) return convert_inertial<DATATYPE>(*Y);
  return pinocchio::InertiaTpl<DATATYPE>::Zero();
}

AssimpLoader::AssimpLoader() : importer(new Assimp::Importer()) {
  // set list of ignored parameters (parameters used for rendering)
  importer->SetPropertyInteger(
      AI_CONFIG_PP_RVC_FLAGS, aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
                                  aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
                                  aiComponent_LIGHTS | aiComponent_CAMERAS |
                                  aiComponent_TEXTURES | aiComponent_TEXCOORDS |
                                  aiComponent_MATERIALS | aiComponent_NORMALS);
}

AssimpLoader::~AssimpLoader() {
  if (importer) delete importer;
}

void AssimpLoader::load(const std::string &file_name) {
  importer->SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
  scene = importer->ReadFile(
      file_name.c_str(),
      aiProcess_SortByPType | aiProcess_Triangulate | aiProcess_RemoveComponent |
          aiProcess_ImproveCacheLocality |
          // TODO: I (Joseph Mirabel) have no idea whether degenerated triangles are
          // properly handled. Enabling aiProcess_FindDegenerates would throw an
          // exception when that happens. Is it too conservative ?
          // aiProcess_FindDegenerates |
          aiProcess_PreTransformVertices | aiProcess_JoinIdenticalVertices);

  if (!scene) {
    const std::string exception_message(std::string("Could not load resource ") +
                                        file_name + std::string("\n") +
                                        importer->GetErrorString() + std::string("\n") +
                                        "Hint: the mesh directory may be wrong.");
    throw std::invalid_argument(exception_message);
  }

  if (!scene->HasMeshes())
    throw std::invalid_argument(std::string("No meshes found in file ") + file_name);

  // cast away the const since we need to give it a name
  const_cast<aiString &>(scene->mName) = file_name;
}

template <typename DATATYPE>
int dfs_build_mesh(const aiScene *scene, const aiNode *node,
                   const Eigen::Matrix<DATATYPE, 3, 1> &scale, int vertices_offset,
                   std::vector<fcl::Vector3<DATATYPE>> &vertices,
                   std::vector<fcl::Triangle> &triangles) {
  if (!node) return 0;

  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode) {
    // Don't convert to y-up orientation, which is what the root node in Assimp does
    if (pnode->mParent != NULL) transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  unsigned nbVertices = 0;
  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh *input_mesh = scene->mMeshes[node->mMeshes[i]];

    // Add the vertices
    DATATYPE max_dim = 0;
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      vertices.push_back(fcl::Vector3<DATATYPE>((DATATYPE)p.x * scale[0],
                                                (DATATYPE)p.y * scale[1],
                                                (DATATYPE)p.z * scale[2]));
      max_dim = std::max({max_dim, std::abs(p.x) * scale[0], std::abs(p.y) * scale[1],
                          std::abs(p.z) * scale[2]});
    }
    if (max_dim < 1e-2 || max_dim > 1e1) {
      print_warning("Mesh ", scene->mName.C_Str(), " has side length ", max_dim,
                    "m which is suspiciously large or small. If this is indeed a unit "
                    "or scaling error, you can set the scale factor in the urdf file.");
    }

    // add the indices
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace &face = input_mesh->mFaces[j];
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

  for (uint32_t i = 0; i < node->mNumChildren; ++i)
    nbVertices += dfs_build_mesh(scene, node->mChildren[i], scale, nbVertices, vertices,
                                 triangles);
  return nbVertices;
}

template <typename DATATYPE>
std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<DATATYPE>>> load_mesh_as_BVH(
    const std::string &mesh_path, const Eigen::Matrix<DATATYPE, 3, 1> &scale) {
  auto loader = AssimpLoader();  // TODO[Xinsong] change to a global loader so we do not
                                 // initialize it every time
  loader.load(mesh_path);

  std::vector<fcl::Vector3<DATATYPE>> vertices;
  std::vector<fcl::Triangle> triangles;

  dfs_build_mesh<DATATYPE>(loader.scene, loader.scene->mRootNode, scale, 0, vertices,
                           triangles);
  // std::cout << "Num of vertex " << nbVertices << " " << vertices.size() << " " <<
  // triangles.size() << std::endl;
  using Model = fcl::BVHModel<fcl::OBBRSS<DATATYPE>>;
  std::shared_ptr<Model> geom = std::make_shared<Model>();
  geom->beginModel();
  geom->addSubModel(vertices, triangles);
  geom->endModel();
  return geom;
}

template <typename DATATYPE>
std::shared_ptr<fcl::Convex<DATATYPE>> load_mesh_as_Convex(
    const std::string &mesh_path, const Eigen::Matrix<DATATYPE, 3, 1> &scale) {
  auto loader = AssimpLoader();
  loader.load(mesh_path);

  std::vector<fcl::Vector3<DATATYPE>> vertices;
  std::vector<fcl::Triangle> triangles;
  /*
  Convex(const std::shared_ptr<const std::vector<Vector3<S>>>& vertices,
          int num_faces, const std::shared_ptr<const std::vector<int>>& faces,
  bool throw_if_invalid = false);
  */
  dfs_build_mesh<DATATYPE>(loader.scene, loader.scene->mRootNode, scale, 0, vertices,
                           triangles);

  auto faces = std::make_shared<std::vector<int>>();
  for (size_t i = 0; i < triangles.size(); i++) {
    faces->push_back(3);
    faces->push_back(triangles[i][0]);
    faces->push_back(triangles[i][1]);
    faces->push_back(triangles[i][2]);
  }
  auto vertices_ptr = std::make_shared<std::vector<fcl::Vector3<DATATYPE>>>(vertices);
  using Convex = fcl::Convex<DATATYPE>;
  auto convex = std::make_shared<Convex>(vertices_ptr, triangles.size(), faces, true);
  return convex;
}

KDL::Vector toKdl(urdf::Vector3 v) { return KDL::Vector(v.x, v.y, v.z); }

KDL::Rotation toKdl(urdf::Rotation r) {
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

KDL::Frame toKdl(urdf::Pose p) {
  return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
}

KDL::Joint toKdl(urdf::JointSharedPtr jnt) {
  KDL::Frame F_parent_jnt = toKdl(jnt->parent_to_joint_origin_transform);
  switch (jnt->type) {
    case urdf::Joint::FIXED:
      return KDL::Joint(jnt->name, KDL::Joint::None);
    case urdf::Joint::REVOLUTE:
      return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * toKdl(jnt->axis),
                        KDL::Joint::RotAxis);
    case urdf::Joint::CONTINUOUS:
      return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * toKdl(jnt->axis),
                        KDL::Joint::RotAxis);
    case urdf::Joint::PRISMATIC:
      return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * toKdl(jnt->axis),
                        KDL::Joint::TransAxis);
    default:
      std::cerr << "Converting unknown joint type of joint " + jnt->name +
                       " into a fixed joint"
                << std::endl;
      return KDL::Joint(jnt->name, KDL::Joint::None);
  }
  return KDL::Joint();
}

KDL::RigidBodyInertia toKdl(urdf::InertialSharedPtr i) {
  KDL::Frame origin = toKdl(i->origin);

  // the mass is frame independent
  double kdl_mass = i->mass;

  // kdl and urdf both specify the com position in the reference frame of the link
  KDL::Vector kdl_com = origin.p;

  // kdl specifies the inertia matrix in the reference frame of the link,
  // while the urdf specifies the inertia matrix in the inertia reference frame
  KDL::RotationalInertia urdf_inertia =
      KDL::RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz);

  // Rotation operators are not defined for rotational inertia,
  // so we use the RigidBodyInertia operators (with com = 0) as a workaround
  KDL::RigidBodyInertia kdl_inertia_wrt_com_workaround =
      origin.M * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), urdf_inertia);

  // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
  // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
  // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
  KDL::RotationalInertia kdl_inertia_wrt_com =
      kdl_inertia_wrt_com_workaround.getRotationalInertia();

  return KDL::RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

// recursive function to walk through tree
bool addChildrenToTree(const urdf::LinkConstSharedPtr &root, KDL::Tree &tree,
                       const bool &verbose) {
  std::vector<urdf::LinkSharedPtr> children = root->child_links;
  if (verbose)
    print_verbose("Link ", root->name, " has ", children.size(), " children");

  // constructs the optional inertia
  KDL::RigidBodyInertia inert(0);
  if (root->inertial) inert = toKdl(root->inertial);
  // constructs the kdl joint
  KDL::Joint jnt = toKdl(root->parent_joint);
  // construct the kdl segment
  KDL::Segment sgm(root->name, jnt,
                   toKdl(root->parent_joint->parent_to_joint_origin_transform), inert);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);
  // recurslively add all children
  for (size_t i = 0; i < children.size(); i++)
    if (!addChildrenToTree(children[i], tree, verbose)) return false;
  return true;
}

bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr &robot_model,
                       KDL::Tree &tree, std::string &tree_root_name,
                       const bool &verbose) {
  if (!robot_model->getRoot()) return false;
  tree_root_name = robot_model->getRoot()->name;
  tree = KDL::Tree(robot_model->getRoot()->name);
  for (size_t i = 0; i < robot_model->getRoot()->child_links.size(); i++)
    if (!addChildrenToTree(robot_model->getRoot()->child_links[i], tree, verbose))
      return false;
  return true;
}
