#include "mplib/collision_detection/fcl/fcl_model.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

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
std::unique_ptr<FCLModelTpl<S>> FCLModelTpl<S>::createFromURDFString(
    const std::string &urdf_string, const std::vector<FCLObjectPtr<S>> &collision_links,
    bool verbose) {
  auto urdf = urdf::parseURDF(urdf_string);
  // package_dir is not needed since urdf_string contains no visual/collision elements
  auto fcl_model = std::make_unique<FCLModelTpl<S>>(urdf, "", verbose, false);

  for (const auto &collision_obj : collision_links) {
    fcl_model->collision_objects_.push_back(collision_obj);
    fcl_model->collision_link_names_.push_back(collision_obj->name);
  }

  // TODO: this should not be needed after switching to FCLObject
  // setLinkOrder with unique collision link names as user_link_names
  auto user_link_names = fcl_model->collision_link_names_;
  auto last = std::unique(user_link_names.begin(), user_link_names.end());
  if (last != user_link_names.end())
    throw std::runtime_error("URDF link names are not unique.");
  // user_link_names.erase(last, user_link_names.end());
  fcl_model->setLinkOrder(fcl_model->collision_link_names_);

  // We assume that the collisions between objects on the same link can be ignored.
  for (size_t i = 0; i < fcl_model->collision_link_names_.size(); i++)
    for (size_t j = 0; j < i; j++)
      if (fcl_model->collision_link_names_[i] != fcl_model->collision_link_names_[j])
        fcl_model->collision_pairs_.push_back(std::make_pair(j, i));

  return fcl_model;
}

template <typename S>
void FCLModelTpl<S>::init(const urdf::ModelInterfaceSharedPtr &urdf_model,
                          const std::string &package_dir) {
  urdf_model_ = urdf_model;
  name_ = urdf_model->getName();
  package_dir_ = package_dir;
  if (not urdf_model_)
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
  urdf::LinkConstSharedPtr root_link = urdf_model_->getRoot();
  dfsParseTree(root_link, "root's parent");

  // TODO: this should not be needed after switching to FCLObject
  // setLinkOrder with unique collision link names as user_link_names
  auto user_link_names = collision_link_names_;
  auto last = std::unique(user_link_names.begin(), user_link_names.end());
  if (last != user_link_names.end())
    throw std::runtime_error("URDF link names are not unique.");
  // user_link_names.erase(last, user_link_names.end());
  setLinkOrder(collision_link_names_);

  for (size_t i = 0; i < collision_link_names_.size(); i++)
    for (size_t j = 0; j < i; j++)
      if (collision_link_names_[i] != collision_link_names_[j])
        collision_pairs_.push_back(std::make_pair(j, i));
}

template <typename S>
void FCLModelTpl<S>::dfsParseTree(const urdf::LinkConstSharedPtr &link,
                                  const std::string &parent_link_name) {
  if (link->collision) {
    auto fcl_obj = std::make_shared<FCLObject<S>>(link->name);
    fcl_obj->pose = Isometry3<S>::Identity();

    for (const auto &geom : link->collision_array) {
      auto geom_model = geom->geometry;
      fcl::CollisionGeometryPtr<S> collision_geometry;
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

      fcl_obj->shapes.push_back(std::make_shared<fcl::CollisionObject<S>>(
          collision_geometry, Isometry3<S>::Identity()));
      fcl_obj->shape_poses.push_back(toIsometry<S>(geom->origin));
    }

    collision_objects_.push_back(fcl_obj);
    collision_link_names_.push_back(link->name);
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
  const std::string extension =
      srdf_filename.substr(srdf_filename.find_last_of('.') + 1);
  if (srdf_filename == "") {
    print_warning("No SRDF file provided!");
    return;
  }
  ASSERT(extension == "srdf", srdf_filename + " does not have the right extension.");

  std::ifstream srdf_stream(srdf_filename.c_str());
  ASSERT(srdf_stream.is_open(), "Cannot open " + srdf_filename);

  std::stringstream buffer;
  buffer << srdf_stream.rdbuf();
  removeCollisionPairsFromSRDFString(buffer.str());
}

template <typename S>
void FCLModelTpl<S>::removeCollisionPairsFromSRDFString(
    const std::string &srdf_string) {
  std::istringstream srdf_stream(srdf_string);

  boost::property_tree::ptree pt;
  boost::property_tree::xml_parser::read_xml(srdf_stream, pt);

  for (const auto &node : pt.get_child("robot"))
    if (node.first == "disable_collisions") {
      const std::string link1 = node.second.get<std::string>("<xmlattr>.link1");
      const std::string link2 = node.second.get<std::string>("<xmlattr>.link2");
      if (verbose_) print_verbose("Try to Remove collision parts: ", link1, " ", link2);
      for (auto iter = collision_pairs_.begin(); iter != collision_pairs_.end();)
        if ((collision_link_names_[iter->first] == link1 &&
             collision_link_names_[iter->second] == link2) ||
            (collision_link_names_[iter->first] == link2 &&
             collision_link_names_[iter->second] == link1))
          iter = collision_pairs_.erase(iter);
        else
          iter++;
    }
}

template <typename S>
void FCLModelTpl<S>::updateCollisionObjects(
    const std::vector<Pose<S>> &link_poses) const {
  for (size_t i = 0; i < collision_objects_.size(); i++) {
    const auto link_pose = link_poses[collision_link_user_indices_[i]].toIsometry();
    const auto &fcl_obj = collision_objects_[i];
    fcl_obj->pose = link_pose;
    for (size_t j = 0; j < fcl_obj->shapes.size(); j++)
      fcl_obj->shapes[j]->setTransform(link_pose * fcl_obj->shape_poses[j]);
  }
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> FCLModelTpl<S>::checkSelfCollision(
    const CollisionRequest &request, const AllowedCollisionMatrixPtr &acm) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  for (const auto &[i, j] : collision_pairs_)
    if (auto type = acm->getAllowedCollision(collision_link_names_[i],
                                             collision_link_names_[j]);
        !type || type == collision_detection::AllowedCollision::NEVER) {
      result.clear();
      collision_detection::fcl::collide(collision_objects_[i], collision_objects_[j],
                                        request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.collision_type = "self";
        tmp.object_name1 = name_;
        tmp.object_name2 = name_;
        tmp.link_name1 = collision_link_names_[i];
        tmp.link_name2 = collision_link_names_[j];
        ret.push_back(tmp);
      }
    }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> FCLModelTpl<S>::checkCollisionWith(
    const FCLModelTplPtr<S> &other, const CollisionRequest &request,
    const AllowedCollisionMatrixPtr &acm) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  for (const auto &col_obj : collision_objects_)
    for (const auto &col_obj2 : other->collision_objects_)
      if (auto type = acm->getAllowedCollision(col_obj->name, col_obj2->name);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        collision_detection::fcl::collide(col_obj, col_obj2, request, result);
        if (result.isCollision()) {
          WorldCollisionResult tmp;
          tmp.res = result;
          tmp.collision_type = "self_articulation";
          tmp.object_name1 = name_;
          tmp.object_name2 = other->name_;
          tmp.link_name1 = col_obj->name;
          tmp.link_name2 = col_obj2->name;
          ret.push_back(tmp);
        }
      }
  return ret;
}

template <typename S>
std::vector<WorldCollisionResultTpl<S>> FCLModelTpl<S>::checkCollisionWith(
    const FCLObjectPtr<S> &object, const CollisionRequest &request,
    const AllowedCollisionMatrixPtr &acm) const {
  std::vector<WorldCollisionResult> ret;
  CollisionResult result;

  for (const auto &col_obj : collision_objects_)
    if (auto type = acm->getAllowedCollision(col_obj->name, object->name);
        !type || type == collision_detection::AllowedCollision::NEVER) {
      result.clear();
      collision_detection::fcl::collide(col_obj, object, request, result);
      if (result.isCollision()) {
        WorldCollisionResult tmp;
        tmp.res = result;
        tmp.collision_type = "self_object";
        tmp.object_name1 = name_;
        tmp.object_name2 = object->name;
        tmp.link_name1 = col_obj->name;
        tmp.link_name2 = object->name;
        ret.push_back(tmp);
      }
    }
  return ret;
}

template <typename S>
WorldDistanceResultTpl<S> FCLModelTpl<S>::distanceSelf(
    const DistanceRequest &request, const AllowedCollisionMatrixPtr &acm) const {
  WorldDistanceResult ret;
  DistanceResult result;

  for (const auto &[i, j] : collision_pairs_)
    if (auto type = acm->getAllowedCollision(collision_link_names_[i],
                                             collision_link_names_[j]);
        !type || type == collision_detection::AllowedCollision::NEVER) {
      result.clear();
      collision_detection::fcl::distance(collision_objects_[i], collision_objects_[j],
                                         request, result);
      if (result.min_distance < ret.min_distance) {
        ret.res = result;
        ret.min_distance = result.min_distance;
        ret.distance_type = "self";
        ret.object_name1 = name_;
        ret.object_name2 = name_;
        ret.link_name1 = collision_link_names_[i];
        ret.link_name2 = collision_link_names_[j];
      }
    }
  return ret;
}

template <typename S>
WorldDistanceResultTpl<S> FCLModelTpl<S>::distanceWith(
    const FCLModelTplPtr<S> &other, const DistanceRequest &request,
    const AllowedCollisionMatrixPtr &acm) const {
  WorldDistanceResult ret;
  DistanceResult result;

  for (const auto &col_obj : collision_objects_)
    for (const auto &col_obj2 : other->collision_objects_)
      if (auto type = acm->getAllowedCollision(col_obj->name, col_obj2->name);
          !type || type == collision_detection::AllowedCollision::NEVER) {
        result.clear();
        collision_detection::fcl::distance(col_obj, col_obj2, request, result);
        if (result.min_distance < ret.min_distance) {
          ret.res = result;
          ret.min_distance = result.min_distance;
          ret.distance_type = "self_articulation";
          ret.object_name1 = name_;
          ret.object_name2 = other->name_;
          ret.link_name1 = col_obj->name;
          ret.link_name2 = col_obj2->name;
        }
      }
  return ret;
}

template <typename S>
WorldDistanceResultTpl<S> FCLModelTpl<S>::distanceWith(
    const FCLObjectPtr<S> &object, const DistanceRequest &request,
    const AllowedCollisionMatrixPtr &acm) const {
  WorldDistanceResult ret;
  DistanceResult result;

  for (const auto &col_obj : collision_objects_)
    if (auto type = acm->getAllowedCollision(col_obj->name, object->name);
        !type || type == collision_detection::AllowedCollision::NEVER) {
      result.clear();
      collision_detection::fcl::distance(col_obj, object, request, result);
      if (result.min_distance < ret.min_distance) {
        ret.res = result;
        ret.min_distance = result.min_distance;
        ret.distance_type = "self_object";
        ret.object_name1 = name_;
        ret.object_name2 = object->name;
        ret.link_name1 = col_obj->name;
        ret.link_name2 = object->name;
      }
    }
  return ret;
}

}  // namespace mplib::collision_detection::fcl
