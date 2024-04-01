#include "mplib/collision_detection/fcl/fcl_model.h"

#include <algorithm>
#include <memory>

#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <urdf_parser/urdf_parser.h>

#include "mplib/collision_detection/fcl/fcl_utils.h"
#include "mplib/macros/assert.h"
#include "mplib/utils/color_printing.h"
#include "mplib/utils/conversion.h"

namespace mplib::collision_detection::fcl {

// Explicit Template Instantiation Definition ==========================================
#define DEFINE_TEMPLATE_FCL_MODEL(S) template class FCLModelTpl<S>

DEFINE_TEMPLATE_FCL_MODEL(float);
DEFINE_TEMPLATE_FCL_MODEL(double);

template <typename S>
FCLModelTpl<S>::FCLModelTpl(const std::string &urdf_filename, bool convex, bool verbose)
    : use_convex_(convex), verbose_(verbose) {
  auto found = urdf_filename.find_last_of("/\\");
  auto urdf_dir = found != urdf_filename.npos ? urdf_filename.substr(0, found) : ".";
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(urdf_filename);
  init(urdf_model, urdf_dir);
}

template <typename S>
FCLModelTpl<S>::FCLModelTpl(const urdf::ModelInterfaceSharedPtr &urdf_model,
                            const std::string &package_dir, bool convex, bool verbose)
    : use_convex_(convex), verbose_(verbose) {
  init(urdf_model, package_dir);
}

template <typename S>
void FCLModelTpl<S>::init(const urdf::ModelInterfaceSharedPtr &urdf_model,
                          const std::string &package_dir) {
  urdf_model_ = urdf_model;
  package_dir_ = package_dir;
  if (not urdf_model_)
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
  urdf::LinkConstSharedPtr root_link = urdf_model_->getRoot();
  dfsParseTree(root_link, "root's parent");
  auto tmp_user_link_names = collision_link_names_;
  auto last = std::unique(tmp_user_link_names.begin(), tmp_user_link_names.end());
  tmp_user_link_names.erase(last, tmp_user_link_names.end());
  setLinkOrder(tmp_user_link_names);

  for (size_t i = 0; i < collision_link_names_.size(); i++)
    for (size_t j = 0; j < i; j++)
      if (collision_link_names_[i] != collision_link_names_[j] &&
          parent_link_names_[i] != collision_link_names_[j] &&
          parent_link_names_[j] != collision_link_names_[i]) {
        // We assume that the collisions between objects append to the same joint can be
        // ignored.
        collision_pairs_.push_back(std::make_pair(j, i));
      }
}

template <typename S>
void FCLModelTpl<S>::dfsParseTree(const urdf::LinkConstSharedPtr &link,
                                  const std::string &parent_link_name) {
  if (link->collision)
    for (const auto &geom : link->collision_array) {
      auto geom_model = geom->geometry;
      fcl::CollisionGeometryPtr<S> collision_geometry;
      auto pose = Isometry3<S>::Identity();
      if (geom_model->type == urdf::Geometry::MESH) {
        const urdf::MeshSharedPtr urdf_mesh =
            urdf::dynamic_pointer_cast<urdf::Mesh>(geom_model);
        std::string file_name = urdf_mesh->filename;
        if (use_convex_ && file_name.find(".convex.stl") == std::string::npos)
          file_name = file_name += ".convex.stl";
        auto mesh_path = (boost::filesystem::path(package_dir_) / file_name).string();
        if (mesh_path == "") {
          std::stringstream ss;
          ss << "Mesh " << file_name << " could not be found.";
          throw std::invalid_argument(ss.str());
        }
        if (verbose_) print_verbose("File name ", file_name);
        Vector3<S> scale {static_cast<S>(urdf_mesh->scale.x),
                          static_cast<S>(urdf_mesh->scale.y),
                          static_cast<S>(urdf_mesh->scale.z)};
        if (use_convex_)
          collision_geometry = loadMeshAsConvex(mesh_path, scale);
        else
          collision_geometry = loadMeshAsBVH(mesh_path, scale);
        if (verbose_) print_verbose(scale, " ", collision_geometry);
      } else if (geom_model->type == urdf::Geometry::CYLINDER) {
        const urdf::CylinderSharedPtr cylinder =
            urdf::dynamic_pointer_cast<urdf::Cylinder>(geom_model);
        collision_geometry = std::make_shared<fcl::Cylinder<S>>(
            static_cast<S>(cylinder->radius), static_cast<S>(cylinder->length));
      } else if (geom_model->type == urdf::Geometry::BOX) {
        const urdf::BoxSharedPtr box =
            urdf::dynamic_pointer_cast<urdf::Box>(geom_model);
        collision_geometry = std::make_shared<fcl::Box<S>>(static_cast<S>(box->dim.x),
                                                           static_cast<S>(box->dim.y),
                                                           static_cast<S>(box->dim.z));
      } else if (geom_model->type == ::urdf::Geometry::SPHERE) {
        const urdf::SphereSharedPtr sphere =
            urdf::dynamic_pointer_cast<urdf::Sphere>(geom_model);
        collision_geometry =
            std::make_shared<fcl::Sphere<S>>(static_cast<S>(sphere->radius));
      } else
        throw std::invalid_argument("Unknown geometry type :");

      if (!collision_geometry)
        throw std::invalid_argument("The polyhedron retrived is empty");
      auto obj {std::make_shared<fcl::CollisionObject<S>>(collision_geometry, pose)};

      collision_objects_.push_back(obj);
      collision_link_names_.push_back(link->name);
      parent_link_names_.push_back(parent_link_name);
      collision_origin2link_poses_.push_back(toIsometry<S>(geom->origin));
    }
  for (const auto &child : link->child_links) dfsParseTree(child, link->name);
}

template <typename S>
void FCLModelTpl<S>::printCollisionPairs() const {
  for (const auto &cp : collision_pairs_)
    print_info(collision_link_names_[cp.first], " ", collision_link_names_[cp.second]);
}

template <typename S>
void FCLModelTpl<S>::setLinkOrder(const std::vector<std::string> &names) {
  user_link_names_ = names;
  collision_link_user_indices_ = {};
  for (size_t i = 0; i < collision_link_names_.size(); i++) {
    if (verbose_) print_verbose(collision_link_names_[i], " ", names[i]);
    auto iter = std::find(names.begin(), names.end(), collision_link_names_[i]);
    if (iter == names.end())
      throw std::invalid_argument("The names does not contain link " +
                                  collision_link_names_[i]);
    collision_link_user_indices_.push_back(iter - names.begin());
  }
}

template <typename S>
void FCLModelTpl<S>::removeCollisionPairsFromSRDF(const std::string &srdf_filename) {
  std::string extension = srdf_filename.substr(srdf_filename.find_last_of('.') + 1);
  if (srdf_filename == "") {
    print_warning("No SRDF file provided!");
    return;
  }

  ASSERT(extension == "srdf", srdf_filename + " does not have the right extension.");

  std::ifstream srdf_stream(srdf_filename.c_str());

  ASSERT(srdf_stream.is_open(), "Cannot open " + srdf_filename);

  boost::property_tree::ptree pt;
  boost::property_tree::xml_parser::read_xml(srdf_stream, pt);

  for (const auto &node : pt.get_child("robot")) {
    if (node.first == "disable_collisions") {
      const std::string link1 = node.second.get<std::string>("<xmlattr>.link1");
      const std::string link2 = node.second.get<std::string>("<xmlattr>.link2");
      if (verbose_) print_verbose("Try to Remove collision parts: ", link1, " ", link2);
      for (auto iter = collision_pairs_.begin(); iter != collision_pairs_.end();) {
        if ((collision_link_names_[iter->first] == link1 &&
             collision_link_names_[iter->second] == link2) ||
            (collision_link_names_[iter->first] == link2 &&
             collision_link_names_[iter->second] == link1)) {
          iter = collision_pairs_.erase(iter);
        } else
          iter++;
      }
    }
  }
}

template <typename S>
void FCLModelTpl<S>::updateCollisionObjects(
    const std::vector<Vector7<S>> &link_pose) const {
  for (size_t i = 0; i < collision_objects_.size(); i++) {
    auto link_i = collision_link_user_indices_[i];
    collision_objects_[i]->setTransform(toIsometry<S>(link_pose[link_i]) *
                                        collision_origin2link_poses_[i]);
  }
}

template <typename S>
void FCLModelTpl<S>::updateCollisionObjects(
    const std::vector<Isometry3<S>> &link_pose) const {
  for (size_t i = 0; i < collision_objects_.size(); i++) {
    auto link_i = collision_link_user_indices_[i];
    collision_objects_[i]->setTransform(link_pose[link_i] *
                                        collision_origin2link_poses_[i]);
  }
}

template <typename S>
bool FCLModelTpl<S>::collide(const fcl::CollisionRequest<S> &request) const {
  fcl::CollisionResult<S> result;
  for (const auto &col_pair : collision_pairs_) {
    fcl::collide(collision_objects_[col_pair.first].get(),
                 collision_objects_[col_pair.second].get(), request, result);
    if (result.isCollision()) return true;
  }
  return false;
}

template <typename S>
std::vector<fcl::CollisionResult<S>> FCLModelTpl<S>::collideFull(
    const fcl::CollisionRequest<S> &request) const {
  // Result will be returned via the collision result structure
  std::vector<fcl::CollisionResult<S>> ret;
  for (const auto &col_pair : collision_pairs_) {
    fcl::CollisionResult<S> result;
    fcl::collide(collision_objects_[col_pair.first].get(),
                 collision_objects_[col_pair.second].get(), request, result);
    ret.push_back(result);
  }
  return ret;
}

}  // namespace mplib::collision_detection::fcl
