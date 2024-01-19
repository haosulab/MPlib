#pragma once

#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../src/fcl_model.h"
#include "../src/macros_utils.hpp"
#include "../src/urdf_utils.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/common/types.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/math/constants.h"
#include "fcl/math/triangle.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/collision_request.h"
#include "fcl/narrowphase/collision_result.h"
#include "fcl/narrowphase/contact.h"
#include "fcl/narrowphase/contact_point.h"
#include "fcl/narrowphase/cost_source.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/distance_request.h"
#include "fcl/narrowphase/distance_result.h"
#include "fcl/narrowphase/gjk_solver_type.h"

using DATATYPE = double;

DEFINE_TEMPLATE_FCL(DATATYPE)

DEFINE_TEMPLATE_EIGEN(DATATYPE)
using FCLModel = FCLModelTpl<DATATYPE>;

/*
using Vector3 = fcl::Vector3<DATATYPE>;
using Vector4 = fcl::VectorN<DATATYPE, 4>;
using Vector7 = fcl::Vector7<DATATYPE>;
using Vector3I = fcl::Vector3<int>;
using Transform = fcl::Transform3<DATATYPE>;
using Matrix = fcl::Matrix3<DATATYPE>;
using Quaternion = fcl::Quaternion<DATATYPE>;


using OBBRSS = fcl::OBBRSS<DATATYPE>;


// Collision Geometry type
using CollisionGeometry = fcl::CollisionGeometry<DATATYPE>;
using Box = fcl::Box<DATATYPE>;
using Sphere = fcl::Sphere<DATATYPE>;
using Capsule = fcl::Capsule<DATATYPE>;
using Cone = fcl::Cone<DATATYPE>;
using Cylinder = fcl::Cylinder<DATATYPE>;
using Convex = fcl::Convex<DATATYPE>;
using Plane = fcl::Plane<DATATYPE>;
using BVHModel_OBBRSS = fcl::BVHModel<OBBRSS>;

// Collision Object = Geometry + Transformation
using CollisionObject = fcl::CollisionObject<DATATYPE>;

// Data type
using Triangle = fcl::Triangle;
*/
namespace py = pybind11;

std::string CollisionGeometry_doc = R"(
    Collision geometry base class.
    This is an FCL class so you can refer to the FCL doc here https://flexible-collision-library.github.io/d6/d5d/classfcl_1_1CollisionGeometry.html)";

std::string Box_doc = R"(
    Box collision geometry.
    Inheriting from CollisionGeometry, this class specializes to a box geometry.)";

std::string Box_constructor_doc = R"(
    Construct a box with given side length.
    
    Args:
        side: side length of the box in an array [x, y, z])";

std::string Box_constructor_doc2 = R"(
    Construct a box with given side length.
    
    Args:
        x: side length of the box in x direction
        y: side length of the box in y direction
        z: side length of the box in z direction)";

std::string Capsule_doc = R"(
    Capsule collision geometry.
    Inheriting from CollisionGeometry, this class specializes to a capsule geometry.)";

std::string Capsule_constructor_doc = R"(
    Construct a capsule with given radius and height.
    
    Args:
        radius: radius of the capsule
        lz: height of the capsule)";

std::string Cylinder_doc = R"(
    Cylinder collision geometry.
    Inheriting from CollisionGeometry, this class specializes to a cylinder geometry.)";

std::string Cylinder_constructor_doc = R"(
    Construct a cylinder with given radius and height.
    
    Args:
        radius: radius of the cylinder
        lz: height of the cylinder)";

std::string OcTree_doc = R"(
    OcTree collision geometry.
    Inheriting from CollisionGeometry, this class specializes to a point cloud geometry represented by an Octree.)";

std::string OcTree_constructor_doc = R"(
    Construct an OcTree with given resolution.
    
    Args:
        resolution: resolution of the OcTree (smallest size of a voxel). you can treat this is as the diameter of a point)";

std::string OcTree_constructor_doc2 = R"(
    Construct an OcTree with given vertices and resolution.
    
    Args:
        vertices: vertices of the point cloud
        resolution: resolution of the OcTree)";

std::string Convex_doc = R"(
    Convex collision geometry.
    Inheriting from CollisionGeometry, this class specializes to a convex geometry.)";

std::string Convex_constructor_doc = R"(
    Construct a convex with given vertices and faces.
    
    Args:
        vertices: vertices of the convex
        num_faces: number of faces of the convex
        faces: faces of the convex geometry represented by a list of vertex indices
        throw_if_invalid: if true, throw an exception if the convex is invalid)";

std::string Convex_constructor_doc2 = R"(
    Construct a convex with given vertices and faces.
    
    Args:
        vertices: vertices of the convex
        faces: faces of the convex geometry represented by a list of vertex indices
        throw_if_invalid: if true, throw an exception if the convex is invalid)";

std::string Convex_get_face_count_doc = R"(
    Get the number of faces of the convex.
    
    Returns:
        number of faces of the convex)";

std::string Convex_get_faces_doc = R"(
    Get the faces of the convex.
    
    Returns:
        faces of the convex represented by a list of vertex indices)";

std::string Convex_get_vertices_doc = R"(
    Get the vertices of the convex.
    
    Returns:
        vertices of the convex)";

std::string Convex_compute_volume_doc = R"(
    Compute the volume of the convex.
    
    Returns:
        volume of the convex)";

std::string Convex_get_interior_point_doc = R"(
    Sample a random interior point of the convex geometry
    
    Returns:
        interior point of the convex)";

std::string BVHModel_OBBRSS_doc = R"(
    BVHModel collision geometry.
    Inheriting from CollisionGeometry, this class specializes to a mesh geometry represented by a BVH tree.)";

std::string BVHModel_OBBRSS_constructor_doc = R"(
    Construct an empty BVHModel.)";

std::string BVHModel_OBBRSS_beginModel_doc = R"(
    Begin to construct a BVHModel.
    
    Args:
        num_faces: number of faces of the mesh
        num_vertices: number of vertices of the mesh)";

std::string BVHModel_OBBRSS_endModel_doc = R"(
    End the construction of a BVHModel.)";

std::string BVHModel_OBBRSS_addSubModel_doc = R"(
    Add a sub-model to the BVHModel.
    
    Args:
        vertices: vertices of the sub-model
        faces: faces of the sub-model represented by a list of vertex indices)";

std::string BVHModel_OBBRSS_get_vertices_doc = R"(
    Get the vertices of the BVHModel.
    
    Returns:
        vertices of the BVHModel)";

std::string BVHModel_OBBRSS_get_faces_doc = R"(
    Get the faces of the BVHModel.
    
    Returns:
        faces of the BVHModel)";

std::string CollisionObject_doc = R"(
    Collision object class.
    This class contains the collision geometry and the transformation of the geometry.)";

std::string CollisionObject_constructor_doc = R"(
    Construct a collision object with given collision geometry and transformation.
    
    Args:
        collision_geometry: collision geometry of the object
        translation: translation of the object
        rotation: rotation of the object)";

std::string FCLModel_doc = R"(
    FCL model class.
    This class contains the collision object and has the ability to perform collision checking and distance computation.)";

std::string FCLModel_constructor_doc = R"(
    Construct an FCL model from URDF and SRDF files.
    
    Args:
        urdf_filename: path to URDF file, can be relative to the current working directory
        verbose: print debug information
        convex: use convex decomposition for collision objects)";

std::string FCLModel_get_collision_pairs_doc = R"(
    Get the collision pairs of the FCL model.
    
    Returns:
        collision pairs of the FCL model. if the FCL model has N collision objects, the collision pairs is a list of N*(N-1)/2 pairs minus the disabled collision pairs)";

std::string FCLModel_get_collision_objects_doc = R"(
    Get the collision objects of the FCL model.
    
    Returns:
        all collision objects of the FCL model)";

std::string FCLModel_set_link_order_doc = R"(
    Set the link order of the FCL model.
    
    Args:
        names: list of link names in the order that you want to set.)";

std::string FCLModel_update_collision_objects_doc = R"(
    Update the collision objects of the FCL model.
    
    Args:
        link_poses: list of link poses in the order of the link order)";

std::string FCLModel_collide_doc = R"(
    Perform collision checking.
    
    Args:
        request: collision request
    
    Returns:
        true if collision happens)";

std::string FCLModel_remove_collision_pairs_from_srdf_doc = R"(
    Remove collision pairs from SRDF.
    
    Args:
        srdf_filename: path to SRDF file, can be relative to the current working directory)";

void build_pyfcl(py::module &m_all) {
  auto m = m_all.def_submodule("fcl");

  // Data type
  auto PyTriangle = py::class_<Triangle, std::shared_ptr<Triangle>>(m, "Triangle");
  PyTriangle.def(py::init())
      .def(py::init<unsigned long, unsigned long, unsigned long>())
      .def("set", [](Triangle &a, const int &p1, const int &p2,
                     const int &p3) { a.set(p1, p2, p3); })
      .def("get", [](Triangle &a, int i) { return a[i]; })
      .def("__getitem__", [](Triangle &a, int i) { return a[i]; });

  // Collision Geometry type
  auto PyCollisionGeometry =
      py::class_<CollisionGeometry, std::shared_ptr<CollisionGeometry>>(
          m, "CollisionGeometry", CollisionGeometry_doc.c_str());
  PyCollisionGeometry.def("computeLocalAABB", &CollisionGeometry::computeLocalAABB)
      .def("isOccupied", &CollisionGeometry::isOccupied)
      .def("isFree", &CollisionGeometry::isFree)
      .def("isUncertain", &CollisionGeometry::isUncertain)
      .def("computeCOM", &CollisionGeometry::computeCOM)
      .def("computeMomentofInertia", &CollisionGeometry::computeMomentofInertia)
      .def("computeVolume", &CollisionGeometry::computeVolume)
      .def("computeMomentofInertiaRelatedToCOM",
           &CollisionGeometry::computeMomentofInertiaRelatedToCOM)
      .def_readwrite("aabb_center", &CollisionGeometry::aabb_center)
      .def_readwrite("aabb_radius", &CollisionGeometry::aabb_radius)
      .def_readwrite("cost_density", &CollisionGeometry::cost_density);

  // collision geometries
  auto PyBox = py::class_<Box, std::shared_ptr<Box>>(m, "Box", PyCollisionGeometry,
                                                     Box_doc.c_str());
  PyBox.def(py::init<const Vector3 &>(), py::arg("side"), Box_constructor_doc.c_str())
      .def(py::init<DATATYPE, DATATYPE, DATATYPE>(), py::arg("x"), py::arg("y"),
           py::arg("z"), Box_constructor_doc2.c_str())
      .def_readwrite("side", &Box::side);

  auto PyCapsule = py::class_<Capsule, std::shared_ptr<Capsule>>(
      m, "Capsule", PyCollisionGeometry, Capsule_doc.c_str());
  PyCapsule
      .def(py::init<DATATYPE, DATATYPE>(), py::arg("radius"), py::arg("lz"),
           Capsule_constructor_doc.c_str())
      .def_readwrite("radius", &Capsule::radius)
      .def_readwrite("lz", &Capsule::lz);

  auto PyCylinder = py::class_<Cylinder, std::shared_ptr<Cylinder>>(
      m, "Cylinder", PyCollisionGeometry, Cylinder_doc.c_str());
  PyCylinder
      .def(py::init<DATATYPE, DATATYPE>(), py::arg("radius"), py::arg("lz"),
           Cylinder_constructor_doc.c_str())
      .def_readwrite("radius", &Cylinder::radius)
      .def_readwrite("lz", &Cylinder::lz);

  auto PyOcTree = py::class_<OcTree, std::shared_ptr<OcTree>>(
      m, "OcTree", PyCollisionGeometry, OcTree_doc.c_str());
  PyOcTree
      .def(py::init<DATATYPE>(), py::arg("resolution"), OcTree_constructor_doc.c_str())
      .def(py::init([](const Matrixx3 &vertices, const double &resolution) {
             octomap::OcTree *tree = new octomap::OcTree(resolution);

             // insert some measurements of occupied cells
             for (auto i = 0; i < vertices.rows(); i++)
               tree->updateNode(
                   octomap::point3d(vertices(i, 0), vertices(i, 1), vertices(i, 2)),
                   true);

             auto tree_ptr = std::shared_ptr<const octomap::OcTree>(tree);
             OcTree *octree = new OcTree(tree_ptr);
             return octree;
           }),
           py::arg("vertices"), py::arg("resolution"), OcTree_constructor_doc2.c_str());

  auto PyConvex = py::class_<Convex, std::shared_ptr<Convex>>(
      m, "Convex", PyCollisionGeometry, Convex_doc.c_str());
  PyConvex
      .def(py::init<const std::shared_ptr<const std::vector<Vector3>> &, int,
                    const std::shared_ptr<const std::vector<int>> &, bool>(),
           py::arg("vertices"), py::arg("num_faces"), py::arg("faces"),
           py::arg("throw_if_invalid") = true, Convex_constructor_doc.c_str())
      .def(py::init([](const Matrixx3 &vertices, const Matrixx3I &faces,
                       const bool &throw_if_invalid) {
             auto vertices_new = std::make_shared<std::vector<Vector3>>();
             auto faces_new = std::make_shared<std::vector<int>>();
             for (auto i = 0; i < vertices.rows(); i++) {
               Vector3 tmp_i;
               tmp_i(0) = vertices(i, 0);
               tmp_i(1) = vertices(i, 1);
               tmp_i(2) = vertices(i, 2);
               vertices_new->push_back(tmp_i);
             }
             for (auto i = 0; i < faces.rows(); i++) {
               faces_new->push_back(3);
               for (size_t j = 0; j < 3; j++) faces_new->push_back(faces(i, j));
             }
             return Convex(vertices_new, faces.rows(), faces_new, throw_if_invalid);
           }),
           py::arg("vertices"), py::arg("faces"), py::arg("throw_if_invalid") = true,
           Convex_constructor_doc2.c_str())
      //.def("radius", &Convex::getRadius)
      .def("get_face_count", &Convex::getFaceCount, Convex_get_face_count_doc.c_str())
      .def("get_faces", &Convex::getFaces, Convex_get_faces_doc.c_str())
      .def("get_vertices", &Convex::getVertices, Convex_get_vertices_doc.c_str())
      .def("compute_volume", &Convex::computeVolume, Convex_compute_volume_doc.c_str())
      .def("get_interior_point", &Convex::getInteriorPoint,
           Convex_get_interior_point_doc.c_str());

  auto PyBVHModel_OBBRSS =
      py::class_<BVHModel_OBBRSS, std::shared_ptr<BVHModel_OBBRSS>>(
          m, "BVHModel", PyCollisionGeometry, BVHModel_OBBRSS_doc.c_str());

  PyBVHModel_OBBRSS.def(py::init<>())
      .def("beginModel", &BVHModel_OBBRSS::beginModel, py::arg("num_faces") = 0,
           py::arg("num_vertices") = 0, BVHModel_OBBRSS_beginModel_doc.c_str())
      .def("endModel", &BVHModel_OBBRSS::endModel, BVHModel_OBBRSS_endModel_doc.c_str())
      .def("addSubModel",
           py::overload_cast<const std::vector<Vector3> &>(
               &BVHModel_OBBRSS::addSubModel),
           py::arg("vertices"), BVHModel_OBBRSS_addSubModel_doc.c_str())
      .def("addSubModel",
           py::overload_cast<const std::vector<Vector3> &,
                             const std::vector<Triangle> &>(
               &BVHModel_OBBRSS::addSubModel),
           py::arg("vertices"), py::arg("faces"),
           BVHModel_OBBRSS_addSubModel_doc.c_str())
      .def(
          "addSubModel",
          [](BVHModel_OBBRSS &a, const std::vector<Vector3> &vertices,
             const std::vector<Vector3I> &faces) {
            std::vector<Triangle> face_list;
            for (size_t i = 0; i < faces.size(); i++)
              face_list.push_back(Triangle(faces[i][0], faces[i][1], faces[i][2]));
            a.addSubModel(vertices, face_list);
          },
          py::arg("vertices"), py::arg("faces"))
      .def(
          "get_vertices",
          [](BVHModel_OBBRSS &a) {
            std::vector<Vector3> ret;
            for (auto i = 0; i < a.num_vertices; i++) ret.push_back(*(a.vertices + i));
            return ret;
          },
          BVHModel_OBBRSS_get_vertices_doc.c_str())
      .def(
          "get_faces",
          [](BVHModel_OBBRSS &a) {
            std::vector<Triangle> ret;
            for (auto i = 0; i < a.num_tris; i++) ret.push_back(*(a.tri_indices + i));
            return ret;
          },
          BVHModel_OBBRSS_get_faces_doc.c_str())
      .def_readonly("num_faces", &BVHModel_OBBRSS::num_tris)
      .def_readonly("num_vertices", &BVHModel_OBBRSS::num_vertices);

  // Collision Object = Geometry + Transformation
  auto PyCollisionObject =
      py::class_<CollisionObject, std::shared_ptr<CollisionObject>>(
          m, "CollisionObject", CollisionObject_doc.c_str());
  PyCollisionObject
      .def(py::init([](const std::shared_ptr<CollisionGeometry> &a, const Vector3 &p,
                       const Vector4 &q) {
             auto q_mat = Quaternion(q(0), q(1), q(2), q(3)).matrix();
             return CollisionObject(a, q_mat, p);
           }),
           py::arg("collision_geometry"), py::arg("translation"), py::arg("rotation"),
           CollisionObject_constructor_doc.c_str())
      .def("get_collision_geometry", &CollisionObject::collisionGeometry)
      .def("get_translation", &CollisionObject::getTranslation)
      .def("get_rotation", &CollisionObject::getRotation)
      .def("set_transformation", [](CollisionObject &a, const Vector7 &pose) {
        Transform3 trans;
        trans.linear() = Quaternion(pose[3], pose[4], pose[5], pose[6]).matrix();
        trans.translation() = pose.head(3);
        a.setTransform(trans);
      });

  /**********    narrowphase    *******/
  auto PyGJKSolverType = py::enum_<GJKSolverType>(m, "GJKSolverType");
  PyGJKSolverType.value("GST_LIBCCD", GJKSolverType::GST_LIBCCD)
      .value("GST_INDEP", GJKSolverType::GST_INDEP)
      .export_values();

  // CollisionRequest
  auto PyCollisionRequest =
      py::class_<CollisionRequest, std::shared_ptr<CollisionRequest>>(
          m, "CollisionRequest");
  PyCollisionRequest
      .def(py::init<size_t, bool, size_t, bool, bool, GJKSolverType, DATATYPE>(),
           py::arg("num_max_contacts") = 1, py::arg("enable_contact") = false,
           py::arg("num_max_cost_sources") = 1, py::arg("enable_cost") = false,
           py::arg("use_approximate_cost") = true,
           py::arg("gjk_solver_type") = GJKSolverType::GST_LIBCCD,
           py::arg("gjk_tolerance") = 1e-6)
      .def("isSatisfied", &CollisionRequest::isSatisfied, py::arg("result"));

  // DistanceRequest
  auto PyDistanceRequest =
      py::class_<DistanceRequest, std::shared_ptr<DistanceRequest>>(m,
                                                                    "DistanceRequest");
  PyDistanceRequest
      .def(py::init<bool, bool, DATATYPE, DATATYPE, DATATYPE, GJKSolverType>(),
           py::arg("enable_nearest_points") = false,
           py::arg("enable_signed_distance") = false, py::arg("rel_err") = 0.0,
           py::arg("abs_err") = 0.0, py::arg("distance_tolerance") = 1e-6,
           py::arg("gjk_solver_type") = GJKSolverType::GST_LIBCCD)
      .def("isSatisfied", &DistanceRequest::isSatisfied, py::arg("result"));

  // DistanceResult Not full suuport
  auto PyDistanceResult =
      py::class_<DistanceResult, std::shared_ptr<DistanceResult>>(m, "DistanceResult");
  PyDistanceResult
      .def(py::init<DATATYPE>(),
           py::arg("min_distance") = std::numeric_limits<DATATYPE>::max())
      .def_readonly("nearest_points", &DistanceResult::nearest_points)
      .def_readonly("min_distance", &DistanceResult::min_distance)
      .def("clear", &DistanceResult::clear);

  // CollisionResult
  auto PyCollisionResult =
      py::class_<CollisionResult, std::shared_ptr<CollisionResult>>(m,
                                                                    "CollisionResult");
  PyCollisionResult.def(py::init<>())
      .def("add_contact", &CollisionResult::addContact, py::arg("c"))
      .def("add_cost_source", &CollisionResult::addCostSource, py::arg("c"),
           py::arg("num_max_cost_sources"))
      .def("is_collision", &CollisionResult::isCollision)
      .def("num_contacts", &CollisionResult::numContacts)
      .def("num_cost_sources", &CollisionResult::numCostSources)
      .def("get_contact", &CollisionResult::getContact, py::arg("i"))
      .def("get_contacts",
           [](CollisionResult &a) {
             std::vector<Contact> contacts;
             a.getContacts(contacts);
             return contacts;
           })
      .def("get_cost_sources",
           [](CollisionResult &a) {
             std::vector<CostSource> cost_sources;
             a.getCostSources(cost_sources);
             return cost_sources;
           })
      .def("clear", &CollisionResult::clear);

  // CostSource Not full support

  auto PyCostSource =
      py::class_<CostSource, std::shared_ptr<CostSource>>(m, "CostSource");
  PyCostSource.def(py::init<>())
      .def(py::init<const Vector3 &, const Vector3 &, DATATYPE>(), py::arg("aabb_min"),
           py::arg("aabb_max"), py::arg("cost_density"))
      .def_readonly("aabb_min", &CostSource::aabb_min)
      .def_readonly("aabb_max", &CostSource::aabb_max)
      .def_readonly("cost_density", &CostSource::cost_density)
      .def_readonly("total_cost", &CostSource::total_cost);

  // ContactPoint Not full support
  auto PyContactPoint =
      py::class_<ContactPoint, std::shared_ptr<ContactPoint>>(m, "ContactPoint");
  PyContactPoint.def(py::init<>())
      .def(py::init<const Vector3 &, const Vector3 &, DATATYPE>(), py::arg("normal"),
           py::arg("pos"), py::arg("penetration_depth"))
      .def_readonly("normal", &ContactPoint::normal)
      .def_readonly("pos", &ContactPoint::pos)
      .def_readonly("penetration_depth", &ContactPoint::penetration_depth);

  // Contact Not full support
  auto PyContact = py::class_<Contact, std::shared_ptr<Contact>>(m, "Contact");
  PyContact.def(py::init<>())
      .def(py::init<const CollisionGeometry *, const CollisionGeometry *, int, int>(),
           py::arg("o1"), py::arg("o2"), py::arg("b1"), py::arg("b2"))
      .def(py::init<const CollisionGeometry *, const CollisionGeometry *, int, int,
                    const Vector3 &, const Vector3 &, DATATYPE>(),
           py::arg("o1"), py::arg("o2"), py::arg("b1"), py::arg("b2"), py::arg("pos"),
           py::arg("normal"), py::arg("depth"))
      .def_readonly("normal", &Contact::normal)
      .def_readonly("pos", &Contact::pos)
      .def_readonly("penetration_depth", &Contact::penetration_depth);

  // Collision function Not full suuport
  m.def("collide",
        [](const CollisionObject *o1, const CollisionObject *o2,
           const CollisionRequest &request) {
          CollisionResult result;
          fcl::collide(o1, o2, request, result);
          return result;
        })
      .def("distance", [](const CollisionObject *o1, const CollisionObject *o2,
                          const DistanceRequest &request) {
        DistanceResult result;
        fcl::distance(o1, o2, request, result);
        return result;
      });

  // FCL model
  auto PyFCLModel = py::class_<FCLModel, std::shared_ptr<FCLModel>>(m, "FCLModel");
  PyFCLModel
      .def(py::init<const std::string &, const bool &, const bool &>(),
           py::arg("urdf_filename"), py::arg("verbose") = true,
           py::arg("convex") = false, FCLModel_constructor_doc.c_str())
      .def("get_collision_pairs", &FCLModel::getCollisionPairs,
           FCLModel_get_collision_pairs_doc.c_str())
      .def("get_collision_objects", &FCLModel::getCollisionObjects,
           FCLModel_get_collision_objects_doc.c_str())
      .def("set_link_order", &FCLModel::setLinkOrder, py::arg("names"),
           FCLModel_set_link_order_doc.c_str())
      .def("update_collision_objects",
           py::overload_cast<const std::vector<Vector7> &>(
               &FCLModel::updateCollisionObjects),
           py::arg("link_poses"), FCLModel_update_collision_objects_doc.c_str())
      .def("collide", &FCLModel::collide, py::arg("request") = CollisionRequest(),
           FCLModel_collide_doc.c_str())
      .def("collide_full", &FCLModel::collideFull,
           py::arg("request") = CollisionRequest())
      .def("get_collision_link_names", &FCLModel::getCollisionLinkNames)
      .def("remove_collision_pairs_from_srdf", &FCLModel::removeCollisionPairsFromSrdf,
           py::arg("srdf_filename"),
           FCLModel_remove_collision_pairs_from_srdf_doc.c_str());

  // Extra function
  m.def("load_mesh_as_BVH", load_mesh_as_BVH<DATATYPE>, py::arg("mesh_path"),
        py::arg("scale"));
  m.def("load_mesh_as_Convex", load_mesh_as_Convex<DATATYPE>, py::arg("mesh_path"),
        py::arg("scale"));
}
