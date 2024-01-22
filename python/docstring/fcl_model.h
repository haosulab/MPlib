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

static const char *__doc_mplib_fcl_FCLModelTpl =
R"doc(FCL collision model of an articulation

See https://github.com/flexible-collision-library/fcl)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_FCLModelTpl =
R"doc(
)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_FCLModelTpl_2 =
R"doc(
Construct an FCL model from URDF and SRDF files.

:param urdf_filename: path to URDF file, can be relative to the current working
    directory
:param verbose: print debug information
:param convex: use convex decomposition for collision objects)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_collide =
R"doc(
Perform collision checking.

:param request: collision request
:return: ``True`` if collision happens)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_collideFull =
R"doc(
)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_getCollisionLinkNames =
R"doc(
)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_getCollisionLinkUserIndices =
R"doc(
)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_getCollisionObjects =
R"doc(
Get the collision objects of the FCL model.

:return: all collision objects of the FCL model)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_getCollisionPairs =
R"doc(
Get the collision pairs of the FCL model.

:return: collision pairs of the FCL model. If the FCL model has N collision
    objects, the collision pairs is a list of N*(N-1)/2 pairs minus the disabled
    collision pairs)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_getUserLinkNames =
R"doc(
)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_printCollisionPairs =
R"doc(
)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_removeCollisionPairsFromSrdf =
R"doc(
Remove collision pairs from SRDF.

:param srdf_filename: path to SRDF file, can be relative to the current working
    directory)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_setLinkOrder =
R"doc(
Set the link order of the FCL model.

:param names: list of link names in the order that you want to set.)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_updateCollisionObjects =
R"doc(
)doc";

static const char *__doc_mplib_fcl_FCLModelTpl_updateCollisionObjects_2 =
R"doc(
Update the collision objects of the FCL model.

:param link_poses: list of link poses in the order of the link order)doc";

/* ----- Begin of custom docstring section ----- */

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
:param lz: height of the capsule)doc";

// ----- fcl::Cylinder ----- //
static const char *__doc_fcl_Cylinder =
R"doc(Cylinder collision geometry.

Inheriting from CollisionGeometry, this class specializes to a cylinder
geometry.)doc";

static const char *__doc_fcl_Cylinder_Cylinder =
R"doc(
Construct a cylinder with given radius and height.

:param radius: radius of the cylinder
:param lz: height of the cylinder)doc";

// ----- fcl::OcTree ----- //
static const char *__doc_fcl_OcTree =
R"doc(OcTree collision geometry.

Inheriting from CollisionGeometry, this class specializes to a point cloud
geometry represented by an OcTree.)doc";

static const char *__doc_fcl_OcTree_OcTree =
R"doc(
Construct an OcTree with given resolution.

:param resolution: resolution of the OcTree (smallest size of a voxel).
    You can treat this is as the diameter of a point.)doc";

static const char *__doc_fcl_OcTree_OcTree_2 =
R"doc(
Construct an OcTree with given vertices and resolution.

:param vertices: vertices of the point cloud
:param resolution: resolution of the OcTree)doc";

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

// ----- fcl::CollisionObject ----- //
static const char *__doc_fcl_CollisionObject =
R"doc(Collision object class.

This class contains the collision geometry and the transformation of the
geometry.)doc";

static const char *__doc_fcl_CollisionObject_CollisionObject =
R"doc(
Construct a collision object with given collision geometry and transformation.

:param collision_geometry: collision geometry of the object
:param translation: translation of the object
:param rotation: rotation of the object)doc";

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
