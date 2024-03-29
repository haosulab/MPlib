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

static const char *__doc_mplib_collision_detection_AllowedCollision = R"doc(AllowedCollision Enum class)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix =
R"doc(AllowedCollisionMatrix for collision checking

All elements in the collision world are referred to by their names. This class
represents which collisions are allowed to happen and which are not.

Mimicking MoveIt2's ``collision_detection::AllowedCollisionMatrix``

https://moveit.picknik.ai/main/api/html/classcollision__detection_1_1AllowedCollisionMatrix.html)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_AllowedCollisionMatrix =
R"doc(
)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_clear =
R"doc(
Clear all data in the allowed collision matrix)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_getAllEntryNames =
R"doc(
Get sorted names of all existing elements (including default_entries_))doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_getAllowedCollision =
R"doc(
Get the type of the allowed collision between two elements

:return: AllowedCollision. This is * ``None`` if the entry does not exist
    (collision is not allowed) * the entry if an entry or a default entry
    exists.)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_getDefaultEntry =
R"doc(
Get the default type of the allowed collision for an element

:param name: name of the element
:return: an AllowedCollision Enum or ``None`` if the default entry does not
    exist)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_getEntry =
R"doc(
Get the type of the allowed collision between two elements

:param name1: name of the first element
:param name2: name of the second element
:return: an AllowedCollision Enum or ``None`` if the entry does not exist.)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_getSize =
R"doc(
Get the size of the allowed collision matrix (number of entries))doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_hasDefaultEntry =
R"doc(
Check if a default entry exists for an element)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_hasEntry =
R"doc(
Check if an entry exists for an element)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_hasEntry_2 =
R"doc(
Check if an entry exists for a pair of elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_print =
R"doc(
Print the allowed collision matrix. "01?-" corresponds to NEVER / ALWAYS /
CONDITIONAL / Entry not found)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_removeDefaultEntry =
R"doc(
Remove the default entry for the element if exists)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_removeDefaultEntry_2 =
R"doc(
Remove the existing default entries for the elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_removeEntry =
R"doc(
Remove the entry for a pair of elements if exists)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_removeEntry_2 =
R"doc(
Remove existing entries between the element and each element in other_names)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_removeEntry_3 =
R"doc(
Remove any existing entries for all possible pairs among two sets of elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_removeEntry_4 =
R"doc(
Remove all entries for all possible pairs between the element and existing
elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_removeEntry_5 =
R"doc(
Remove all entries for all possible pairs between each of the elements and
existing elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setDefaultEntry =
R"doc(
Set the default value for entries that include name but are not set explicitly
with setEntry(). Apply to future changes of the element set.)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setDefaultEntry_2 =
R"doc(
Set the default entries for the elements. Apply to future changes of the element
set.)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setEntry =
R"doc(
Set an entry for a pair of elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setEntry_2 =
R"doc(
Set the entries between the element and each element in other_names)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setEntry_3 =
R"doc(
Set the entries for all possible pairs among two sets of elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setEntry_4 =
R"doc(
Set the entries for all possible pairs between the element and existing
elements. As the set of elements might change in the future, consider using
setDefaultEntry() instead.)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setEntry_5 =
R"doc(
Set the entries for all possible pairs between each of the elements and existing
elements. As the set of elements might change in the future, consider using
setDefaultEntry() instead.)doc";

static const char *__doc_mplib_collision_detection_AllowedCollisionMatrix_setEntry_6 =
R"doc(
Set the entries for all possible pairs among all existing elements)doc";

static const char *__doc_mplib_collision_detection_AllowedCollision_ALWAYS = R"doc(Collision is always allowed)doc";

static const char *__doc_mplib_collision_detection_AllowedCollision_CONDITIONAL = R"doc(Collision contact is allowed depending on a predicate)doc";

static const char *__doc_mplib_collision_detection_AllowedCollision_NEVER = R"doc(Collision is never allowed)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
