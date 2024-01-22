#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "sample_header.h"

// Symbol: mkdoc_doc
constexpr struct /* mkdoc_doc */ {
  // Symbol: mplib
  struct /* mplib */ {
    // Symbol: mplib::PlanningWorld
    struct /* PlanningWorld */ {
      // Source: sample_header.h:17
      const char* doc =
R"""(PlanningWorld

- Begin first unordered list element. Volutpat blandit aliquam etiam erat
  velit scelerisque. End first unordered list element.
- Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
  Ipsum dolor sit amet consectetur adipiscing. End second unordered list
  element.
- Begin third unordered list element. Hac habitasse platea dictumst quisque
  sagittis purus sit. End third unordered list element.)""";
      // Symbol: mplib::PlanningWorld::attachObject
      struct /* attachObject */ {
        // Source: sample_header.h:36
        const char* doc =
R"""(Attaches existing normal object to specified link of articulation.

If the object is currently attached, disallow collision between the
object and previous touch_links. Updates acm_ to allow collisions
between attached object and touch_links.

Parameter ``name:``:
    normal object name to attach

Parameter ``art_name:``:
    name of the planned articulation to attach to

Parameter ``link_id:``:
    index of the link of the planned articulation to attach to. Begin
    precondition. Cras fermentum odio eu feugiat pretium nibh.

Parameter ``pose:``:
    attached pose (relative pose from attached link to object)

Parameter ``touch_links:``:
    link names that the attached object touches

Returns:
    : the attached object

Raises:
    ValueError if normal object with given name does not exist or if
    planned articulation with given name does not exist)""";
      } attachObject;
    } PlanningWorld;
  } mplib;
} mkdoc_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
