#pragma once

#include <string>

#include <kdl/tree.hpp>
#include <urdf_world/types.h>

namespace mplib::kinematics::kdl {

bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr &urdf_model, KDL::Tree &tree,
                       std::string &tree_root_name, bool verbose = false);

}  // namespace mplib::kinematics::kdl
