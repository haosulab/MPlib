#pragma once

/*
  This file contains docstrings for use in the Python bindings.
  Do not edit! They were automatically extracted by mkdoc.py.
 */

#define __EXPAND(x)                                      x
#define __COUNT(_1, _2, _3, _4, _5, _6, _7, COUNT, ...)  COUNT
#define __VA_SIZE(...)                                   __EXPAND(__COUNT(__VA_ARGS__, 7, 6, 5, 4, 3, 2, 1, 0))
#define __CAT1(a, b)                                     a ## b
#define __CAT2(a, b)                                     __CAT1(a, b)
#define __DOC1(n1)                                       __doc_##n1
#define __DOC2(n1, n2)                                   __doc_##n1##_##n2
#define __DOC3(n1, n2, n3)                               __doc_##n1##_##n2##_##n3
#define __DOC4(n1, n2, n3, n4)                           __doc_##n1##_##n2##_##n3##_##n4
#define __DOC5(n1, n2, n3, n4, n5)                       __doc_##n1##_##n2##_##n3##_##n4##_##n5
#define __DOC6(n1, n2, n3, n4, n5, n6)                   __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6
#define __DOC7(n1, n2, n3, n4, n5, n6, n7)               __doc_##n1##_##n2##_##n3##_##n4##_##n5##_##n6##_##n7
#define DOC(...)                                         __EXPAND(__EXPAND(__CAT2(__DOC, __VA_SIZE(__VA_ARGS__)))(__VA_ARGS__))

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/* ----- Begin of custom docstring section ----- */

// ----- fcl::Triangle ----- //
static const char *__doc_fcl_Triangle =
R"doc(Triangle with 3 indices for points.

This is an FCL class so you can refer to the FCL doc here.
https://flexible-collision-library.github.io/de/daa/classfcl_1_1Triangle.html)doc";

// ----- fcl::CollisionGeometry ----- //
static const char *__doc_fcl_CollisionGeometry =
R"doc(Collision geometry base class.

This is an FCL class so you can refer to the FCL doc here.
https://flexible-collision-library.github.io/d6/d5d/classfcl_1_1CollisionGeometry.html)doc";

// ----- fcl::Box ----- //
static const char *__doc_fcl_Box =
R"doc(Box collision geometry.

Inheriting from CollisionGeometry, this class specializes to a box geometry.)doc";

static const char *__doc_fcl_Box_Box =
R"doc(
Construct a box with given side length.

:param side: side length of the box in an array [x, y, z])doc";

static const char *__doc_fcl_Box_Box_2 =
R"doc(
Construct a box with given side length.

:param x: side length of the box in x direction
:param y: side length of the box in y direction
:param z: side length of the box in z direction)doc";

// ----- fcl::Capsule ----- //
static const char *__doc_fcl_Capsule =
R"doc(Capsule collision geometry.

Inheriting from CollisionGeometry, this class specializes to a capsule geometry.)doc";

static const char *__doc_fcl_Capsule_Capsule =
R"doc(
Construct a capsule with given radius and height.

:param radius: radius of the capsule
:param lz: height of the capsule along z axis)doc";

// ----- fcl::Cone ----- //
static const char *__doc_fcl_Cone =
R"doc(Cone collision geometry.

Inheriting from CollisionGeometry, this class specializes to a cone geometry.)doc";

static const char *__doc_fcl_Cone_Cone =
R"doc(
Construct a cone with given radius and height.

:param radius: radius of the cone
:param lz: height of the cone along z axis)doc";

// ----- fcl::Convex ----- //
static const char *__doc_fcl_Convex =
R"doc(Convex collision geometry.

Inheriting from CollisionGeometry, this class specializes to a convex geometry.)doc";

static const char *__doc_fcl_Convex_Convex =
R"doc(
Construct a convex with given vertices and faces.

:param vertices: vertices of the convex
:param num_faces: number of faces of the convex
:param faces: faces of the convex geometry represented by a list of vertex
    indices
:param throw_if_invalid: if ``True``, throw an exception if the convex is
    invalid)doc";

static const char *__doc_fcl_Convex_Convex_2 =
R"doc(
Construct a convex with given vertices and faces.

:param vertices: vertices of the convex
:param faces: faces of the convex geometry represented by a list of vertex
    indices
:param throw_if_invalid: if ``True``, throw an exception if the convex is
    invalid)doc";

static const char *__doc_fcl_Convex_getFaceCount =
R"doc(
Get the number of faces of the convex.

:return: number of faces of the convex)doc";

static const char *__doc_fcl_Convex_getFaces =
R"doc(
Get the faces of the convex.

:return: faces of the convex represented by a list of vertex indices)doc";

static const char *__doc_fcl_Convex_getVertices =
R"doc(
Get the vertices of the convex.

:return: vertices of the convex)doc";

static const char *__doc_fcl_Convex_computeVolume =
R"doc(
Compute the volume of the convex.

:return: volume of the convex)doc";

static const char *__doc_fcl_Convex_getInteriorPoint =
R"doc(
Sample a random interior point of the convex geometry

:return: interior point of the convex)doc";

// ----- fcl::Cylinder ----- //
static const char *__doc_fcl_Cylinder =
R"doc(Cylinder collision geometry.

Inheriting from CollisionGeometry, this class specializes to a cylinder
geometry.)doc";

static const char *__doc_fcl_Cylinder_Cylinder =
R"doc(
Construct a cylinder with given radius and height.

:param radius: radius of the cylinder
:param lz: height of the cylinder along z axis)doc";

// ----- fcl::Ellipsoid ----- //
static const char *__doc_fcl_Ellipsoid =
R"doc(Ellipsoid collision geometry.

Inheriting from CollisionGeometry, this class specializes to a ellipsoid
geometry.)doc";

static const char *__doc_fcl_Ellipsoid_Ellipsoid =
R"doc(
Construct a ellipsoid with given parameters.

:param a: length of the ``x`` semi-axis
:param b: length of the ``y`` semi-axis
:param c: length of the ``z`` semi-axis)doc";

static const char *__doc_fcl_Ellipsoid_Ellipsoid_2 =
R"doc(
Construct a ellipsoid with given parameters.

:param radii: vector of the length of the ``x``, ``y``, and ``z`` semi-axes)doc";

// ----- fcl::Halfspace ----- //
static const char *__doc_fcl_Halfspace =
R"doc(Infinite halfspace collision geometry.

Inheriting from CollisionGeometry, this class specializes to a halfspace geometry.)doc";

static const char *__doc_fcl_Halfspace_Halfspace =
R"doc(
Construct a halfspace with given normal direction and offset where ``n * p = d``.
Points in the negative side of the separation plane ``{p | n * p < d}`` are inside
the half space (will have collision).

:param n: normal direction of the halfspace
:param d: offset of the halfspace)doc";

static const char *__doc_fcl_Halfspace_Halfspace_2 =
R"doc(
Construct a halfspace with given halfspace parameters where ``ax + by + cz = d``.
Points in the negative side of the separation plane ``{(x, y, z) | ax + by + cz < d}``
are inside the half space (will have collision).)doc";

static const char *__doc_fcl_Halfspace_signedDistance =
R"doc(
Compute the signed distance of a point to the halfspace as ``n * p - d``.

:param p: a point in 3D space
:return: signed distance of the point to the halfspace)doc";

static const char *__doc_fcl_Halfspace_distance =
R"doc(
Compute the distance of a point to the halfspace as ``abs(n * p - d)``.

:param p: a point in 3D space
:return: distance of the point to the halfspace)doc";

// ----- fcl::Plane ----- //
static const char *__doc_fcl_Plane =
R"doc(Infinite plane collision geometry.

Inheriting from CollisionGeometry, this class specializes to a plane geometry.)doc";

static const char *__doc_fcl_Plane_Plane =
R"doc(
Construct a plane with given normal direction and offset where ``n * p = d``.

:param n: normal direction of the plane
:param d: offset of the plane)doc";

static const char *__doc_fcl_Plane_Plane_2 =
R"doc(
Construct a plane with given plane parameters where ``ax + by + cz = d``.)doc";

static const char *__doc_fcl_Plane_signedDistance =
R"doc(
Compute the signed distance of a point to the plane as ``n * p - d``.

:param p: a point in 3D space
:return: signed distance of the point to the plane)doc";

static const char *__doc_fcl_Plane_distance =
R"doc(
Compute the distance of a point to the plane as ``abs(n * p - d)``.

:param p: a point in 3D space
:return: distance of the point to the plane)doc";

// ----- fcl::Sphere ----- //
static const char *__doc_fcl_Sphere =
R"doc(Sphere collision geometry.

Inheriting from CollisionGeometry, this class specializes to a sphere geometry.)doc";

static const char *__doc_fcl_Sphere_Sphere =
R"doc(
Construct a sphere with given radius.

:param radius: radius of the sphere)doc";

// ----- fcl::TriangleP ----- //
static const char *__doc_fcl_TriangleP =
R"doc(TriangleP collision geometry.

Inheriting from CollisionGeometry, this class specializes to a triangleP geometry.)doc";

static const char *__doc_fcl_TriangleP_TriangleP =
R"doc(
Construct a set of triangles from vectors of point coordinates.

:param a: vector of point ``x`` coordinates
:param b: vector of point ``y`` coordinates
:param c: vector of point ``z`` coordinates)doc";

// ----- fcl::BVHModel<fcl::OBBRSS> ----- //
static const char *__doc_fcl_BVHModel_OBBRSS =
R"doc(BVHModel collision geometry.

Inheriting from CollisionGeometry, this class specializes to a mesh geometry
represented by a BVH tree.)doc";

static const char *__doc_fcl_BVHModel_OBBRSS_beginModel =
R"doc(
Begin to construct a BVHModel.

:param num_faces: number of faces of the mesh
:param num_vertices: number of vertices of the mesh)doc";

static const char *__doc_fcl_BVHModel_OBBRSS_endModel =
R"doc(
End the construction of a BVHModel.)doc";

static const char *__doc_fcl_BVHModel_OBBRSS_addSubModel =
R"doc(
Add a sub-model to the BVHModel.

:param vertices: vertices of the sub-model)doc";

static const char *__doc_fcl_BVHModel_OBBRSS_addSubModel_2 =
R"doc(
Add a sub-model to the BVHModel.

:param vertices: vertices of the sub-model
:param faces: faces of the sub-model represented by a list of vertex indices)doc";

static const char *__doc_fcl_BVHModel_OBBRSS_addSubModel_3 =
R"doc(
Add a sub-model to the BVHModel.

:param vertices: vertices of the sub-model
:param faces: faces of the sub-model represented by a list of vertex indices)doc";

static const char *__doc_fcl_BVHModel_OBBRSS_getVertices =
R"doc(
Get the vertices of the BVHModel.

:return: vertices of the BVHModel)doc";

static const char *__doc_fcl_BVHModel_OBBRSS_getFaces =
R"doc(
Get the faces of the BVHModel.

:return: faces of the BVHModel)doc";

// ----- fcl::OcTree ----- //
static const char *__doc_fcl_OcTree =
R"doc(OcTree collision geometry.

Inheriting from CollisionGeometry, this class specializes to a point cloud
geometry represented by an OcTree.)doc";

static const char *__doc_fcl_OcTree_OcTree =
R"doc(
Construct an OcTree with given resolution.

:param resolution: resolution of the OcTree (smallest size of a voxel).
    You can treat this is as the diameter of a point. Default is 0.01.)doc";

static const char *__doc_fcl_OcTree_OcTree_2 =
R"doc(
Construct an OcTree with given vertices and resolution.

:param vertices: vertices of the point cloud
:param resolution: resolution of the OcTree. Default is 0.01)doc";

// ----- fcl::CollisionObject ----- //
static const char *__doc_fcl_CollisionObject =
R"doc(Collision object class.

This class contains the collision geometry and the transformation of the
geometry.)doc";

static const char *__doc_fcl_CollisionObject_CollisionObject =
R"doc(
Construct a collision object with given collision geometry and transformation.

:param collision_geometry: collision geometry of the object
:param pose: pose of the object)doc";

static const char *__doc_fcl_CollisionObject_pose =
R"doc(
Pose of the collision object in world)doc";

static const char *__doc_fcl_CollisionObject_set_pose =
R"doc(
Sets the pose of the collision object in world

:param pose: New pose of the collision object)doc";

static const char *__doc_fcl_CollisionObject_get_pose =
R"doc(
Gets the current pose of the collision object in world

:return: The current pose of the collision object)doc";

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
