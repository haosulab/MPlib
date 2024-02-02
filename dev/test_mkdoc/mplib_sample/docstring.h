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

static const char *__doc_mplib_PlanningWorldTpl =
R"doc(PlanningWorld

- Begin first unordered list element. Volutpat blandit aliquam etiam erat
  velit scelerisque. End first unordered list element.
- Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
  Ipsum dolor sit amet consectetur adipiscing. End second unordered list
  element.
- Begin third unordered list element. Hac habitasse platea dictumst quisque
  sagittis purus sit. End third unordered list element.)doc";

static const char *__doc_mplib_PlanningWorldTpl_attachObject =
R"doc(
Attaches existing normal object to specified link of articulation.

If the object is currently attached, disallow collision between the object and
previous touch_links. Updates acm_ to allow collisions between attached object
and touch_links.

:param urdf_filename: path to URDF file, can be relative to the current working
    directory
:param name: normal object name to attach
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to.
    Begin precondition. Cras fermentum odio eu feugiat pretium nibh.
:param pose: attached pose (relative pose from attached link to object)
:param touch_links: link names that the attached object touches
:return: the attached object
:raises ValueError: if normal object with given name does not exist or if
    planned articulation with given name does not exist)doc";

static const char *__doc_mplib_compoundstate2vector =
R"doc(
)doc";

/* ----- Begin of custom docstring section ----- */

static const char *__doc_mplib_PlanningWorld_doc2 =
R"doc(
PlanningWorld, custom docstring should not change)doc";

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
