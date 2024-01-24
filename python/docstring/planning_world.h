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

static const char *__doc_mplib_PlanningWorldTpl = R"doc(Planning world for collision checking)doc";

static const char *__doc_mplib_PlanningWorldTpl_PlanningWorldTpl =
R"doc(
Constructs a PlanningWorld with given articulations and normal objects

:param articulations: list of articulated models
:param articulation_names: name of the articulated models
:param normal_objects: list of collision objects that are not articulated
:param normal_object_names: name of the normal objects
:param plan_articulation_id: id of the articulated model that is used for
    planning)doc";

static const char *__doc_mplib_PlanningWorldTpl_addArticulation =
R"doc(
Add an articulated model to the planning world.

:param model: articulated model to be added
:param name: name of the articulated model)doc";

static const char *__doc_mplib_PlanningWorldTpl_addArticulations =
R"doc(
Add a list of articulated models to the planning world.

:param models: list of articulated models to be added
:param names: list of names of the articulated models)doc";

static const char *__doc_mplib_PlanningWorldTpl_collide =
R"doc(
Check collision in the planning world.

:return: ``True`` if collision exists)doc";

static const char *__doc_mplib_PlanningWorldTpl_collideFull =
R"doc(
Check collision between the articulated model and all objects.

:param index: index of the articulated model
:param request: collision request params. Can leave empty for default value
:return: List of WorldCollisionResult objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_collideWithOthers =
R"doc(
Check collision between the articulated model and other objects.

:param index: index of the articulated model
:param request: collision request params. Can leave empty for default value
:return: List of WorldCollisionResult objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_getArticulationNames =
R"doc(
Get the names of articulated models.

:return: list of names of articulated models)doc";

static const char *__doc_mplib_PlanningWorldTpl_getArticulations =
R"doc(
Get the list of articulated models.

:return: list of articulated models)doc";

static const char *__doc_mplib_PlanningWorldTpl_getMoveArticulationId =
R"doc(
)doc";

static const char *__doc_mplib_PlanningWorldTpl_getNormalObjectNames =
R"doc(
Get the names of non-articulated collision objects.

:return: list of names of non-articulated collision objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_getNormalObjects =
R"doc(
Get the list of non-articulated collision objects.

:return: list of non-articulated collision objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_printAttachedToolPose =
R"doc(
Print the pose of the attached tool.)doc";

static const char *__doc_mplib_PlanningWorldTpl_removeAttach =
R"doc(
Remove attach object so there won't be anything on the end effector when
``use_attach`` is set to ``True`` again)doc";

static const char *__doc_mplib_PlanningWorldTpl_removeNormalObject =
R"doc(
Remove am non-articulated object

:param name: name of the non-articulated collision object
:return: ``True`` if the item exists and ``False`` otherwise)doc";

static const char *__doc_mplib_PlanningWorldTpl_selfCollide =
R"doc(
Check collision between the articulated model and itself.

:param index: index of the articulated model
:param request: collision request params. Can leave empty for default value
:return: List of WorldCollisionResult objects)doc";

static const char *__doc_mplib_PlanningWorldTpl_setMoveArticulationId =
R"doc(
)doc";

static const char *__doc_mplib_PlanningWorldTpl_setNormalObject =
R"doc(
Add a non-articulated collision object to the planning world.

:param name: name of the non-articulated collision object
:param collision_object: the non-articulated collision object to be added)doc";

static const char *__doc_mplib_PlanningWorldTpl_setQpos =
R"doc(
Set the joint qpos of the articulated model.

:param index: index of the articulated model
:param qpos: joint angles of the *movegroup only*)doc";

static const char *__doc_mplib_PlanningWorldTpl_setQposAll =
R"doc(
Set the joint qpos of all articulated models.

:param qpos: joint angles of all the models (*movegroup only*))doc";

static const char *__doc_mplib_PlanningWorldTpl_setUseAttach =
R"doc(
Set whether to use attached tool for collision checking.

:param use: whether to use attached tool)doc";

static const char *__doc_mplib_PlanningWorldTpl_setUsePointCloud =
R"doc(
Set whether to use point cloud for collision checking.

:param use: whether to use point cloud)doc";

static const char *__doc_mplib_PlanningWorldTpl_updateAttachedBox =
R"doc(
Add a box as the attached tool.

:param size: size of the box, [size_x, size_y, size_z]
:param link_id: link id of the attached box
:param pose: pose of the attached box w.r.t. the link it's attached to. [x, y,
    z, qw, qx, qy, qz])doc";

static const char *__doc_mplib_PlanningWorldTpl_updateAttachedMesh =
R"doc(
Add a mesh as the attached tool.

:param mesh_path: path to the mesh file
:param link_id: link id of the attached mesh
:param pose: pose of the attached mesh w.r.t. the link it's attached to. [x, y,
    z, qw, qx, qy, qz])doc";

static const char *__doc_mplib_PlanningWorldTpl_updateAttachedSphere =
R"doc(
Add a sphere as the attached tool.

:param radius: radius of the sphere
:param link_id: link id of the attached sphere
:param pose: pose of the attached sphere w.r.t. the link it's attached to. [x,
    y, z, qw, qx, qy, qz])doc";

static const char *__doc_mplib_PlanningWorldTpl_updateAttachedTool =
R"doc(
Attach or update the attached object

:param p_geom: fcl collision geometry of the attached tool
:param link_id: id of the link to which the object is attached
:param pose: pose of the attached object w.r.t. the link it's attached to. [x,
    y, z, qw, qx, qy, qz])doc";

static const char *__doc_mplib_PlanningWorldTpl_updatePointCloud =
R"doc(
Update the point cloud for collision checking.

:param vertices: vertices of the point cloud
:param radius: radius of each point in the point cloud)doc";

static const char *__doc_mplib_PlanningWorldTpl_use_attach = R"doc()doc";

static const char *__doc_mplib_PlanningWorldTpl_use_point_cloud = R"doc()doc";

static const char *__doc_mplib_WorldCollisionResultTpl = R"doc(Result of the collision checking.)doc";

static const char *__doc_mplib_WorldCollisionResultTpl_collision_type = R"doc(type of the collision)doc";

static const char *__doc_mplib_WorldCollisionResultTpl_link_name1 = R"doc(link name of the first object in collision)doc";

static const char *__doc_mplib_WorldCollisionResultTpl_link_name2 = R"doc(link name of the second object in collision)doc";

static const char *__doc_mplib_WorldCollisionResultTpl_object_name1 = R"doc(name of the first object)doc";

static const char *__doc_mplib_WorldCollisionResultTpl_object_name2 = R"doc(name of the second object)doc";

static const char *__doc_mplib_WorldCollisionResultTpl_res = R"doc(the fcl CollisionResult)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
