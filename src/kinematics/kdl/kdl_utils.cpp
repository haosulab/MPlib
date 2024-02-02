#include "mplib/kinematics/kdl/kdl_utils.h"

#include <urdf_model/link.h>
#include <urdf_model/model.h>

#include "mplib/utils/color_printing.h"

namespace mplib::kinematics::kdl {

KDL::Vector toKDL(const urdf::Vector3 &v) { return KDL::Vector(v.x, v.y, v.z); }

KDL::Rotation toKDL(const urdf::Rotation &r) {
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

KDL::Frame toKDL(const urdf::Pose &p) {
  return KDL::Frame(toKDL(p.rotation), toKDL(p.position));
}

KDL::Joint toKDL(const urdf::JointSharedPtr &jnt) {
  const KDL::Frame F_parent_jnt = toKDL(jnt->parent_to_joint_origin_transform);
  switch (jnt->type) {
    case urdf::Joint::FIXED:
      return KDL::Joint(jnt->name, KDL::Joint::None);
    case urdf::Joint::REVOLUTE:
      return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * toKDL(jnt->axis),
                        KDL::Joint::RotAxis);
    case urdf::Joint::CONTINUOUS:
      return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * toKDL(jnt->axis),
                        KDL::Joint::RotAxis);
    case urdf::Joint::PRISMATIC:
      return KDL::Joint(jnt->name, F_parent_jnt.p, F_parent_jnt.M * toKDL(jnt->axis),
                        KDL::Joint::TransAxis);
    default:
      std::cerr << "Converting unknown joint type of joint " + jnt->name +
                       " into a fixed joint"
                << std::endl;
      return KDL::Joint(jnt->name, KDL::Joint::None);
  }
}

KDL::RigidBodyInertia toKDL(const urdf::InertialSharedPtr &i) {
  const KDL::Frame origin = toKDL(i->origin);

  // the mass is frame independent
  const double kdl_mass = i->mass;

  // kdl and urdf both specify the com position in the reference frame of the link
  const KDL::Vector kdl_com = origin.p;

  // kdl specifies the inertia matrix in the reference frame of the link,
  // while the urdf specifies the inertia matrix in the inertia reference frame
  const KDL::RotationalInertia urdf_inertia =
      KDL::RotationalInertia(i->ixx, i->iyy, i->izz, i->ixy, i->ixz, i->iyz);

  // Rotation operators are not defined for rotational inertia,
  // so we use the RigidBodyInertia operators (with com = 0) as a workaround
  const KDL::RigidBodyInertia kdl_inertia_wrt_com_workaround =
      origin.M * KDL::RigidBodyInertia(0, KDL::Vector::Zero(), urdf_inertia);

  // Note that the RigidBodyInertia constructor takes the 3d inertia wrt the com
  // while the getRotationalInertia method returns the 3d inertia wrt the frame origin
  // (but having com = Vector::Zero() in kdl_inertia_wrt_com_workaround they match)
  const KDL::RotationalInertia kdl_inertia_wrt_com =
      kdl_inertia_wrt_com_workaround.getRotationalInertia();

  return KDL::RigidBodyInertia(kdl_mass, kdl_com, kdl_inertia_wrt_com);
}

// recursive function to walk through tree
bool addChildrenToTree(const urdf::LinkConstSharedPtr &root, KDL::Tree &tree,
                       bool verbose) {
  const std::vector<urdf::LinkSharedPtr> children = root->child_links;
  if (verbose)
    print_verbose("Link ", root->name, " has ", children.size(), " children");

  // constructs the optional inertia
  KDL::RigidBodyInertia inert(0);
  if (root->inertial) inert = toKDL(root->inertial);
  // constructs the kdl joint
  const KDL::Joint jnt = toKDL(root->parent_joint);
  // construct the kdl segment
  const KDL::Segment sgm(root->name, jnt,
                         toKDL(root->parent_joint->parent_to_joint_origin_transform),
                         inert);

  // add segment to tree
  tree.addSegment(sgm, root->parent_joint->parent_link_name);
  // recurslively add all children
  for (const auto &child : children)
    if (!addChildrenToTree(child, tree, verbose)) return false;
  return true;
}

bool treeFromUrdfModel(const urdf::ModelInterfaceSharedPtr &urdf_model, KDL::Tree &tree,
                       std::string &tree_root_name, bool verbose) {
  const urdf::LinkConstSharedPtr root_link = urdf_model->getRoot();
  if (!root_link) return false;

  tree_root_name = root_link->name;
  tree = KDL::Tree(tree_root_name);
  for (const auto &child_link : root_link->child_links)
    if (!addChildrenToTree(child_link, tree, verbose)) return false;
  return true;
}

}  // namespace mplib::kinematics::kdl
