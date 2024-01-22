#pragma once

/// @dir
/// Directory documentation is ignored. Sit amet massa vitae tortor. Pulvinar
/// pellentesque habitant morbi tristique senectus et. Lacus sed turpis
/// tincidunt id.

/// @file
/// File documentation is ignored. Sit amet nisl purus in mollis nunc sed id
/// semper. Risus nec feugiat in fermentum posuere urna nec tincidunt praesent.
/// Suscipit tellus mauris a diam.

/// @defgroup first_group Elementum pulvinar etiam non quam lacus.
/// Ultrices in iaculis nunc sed augue lacus viverra. Dolor sit amet
/// consectetur adipiscing elit duis tristique.

#include <string>
#include <vector>


/// @def PREPROCESSOR_DEFINITION
/// Preprocessor definitions are ignored. In nibh mauris cursus mattis
/// molestie a. Non arcu risus quis varius quam quisque id.
#define PREPROCESSOR_DEFINITION "Nisl purus in mollis nunc sed id."

/// Root-level symbol. Magna fermentum iaculis eu non diam phasellus
/// vestibulum.
struct RootLevelSymbol {};

/// @namespace drake
/// Namespaces are ignored. Enim blandit volutpat maecenas volutpat blandit. Eu
/// feugiat pretium nibh ipsum consequat.
namespace drake {

/**
 * 1. Begin first ordered list element. Rutrum quisque non tellus orci ac
 *    auctor. End first ordered list element.
 * 2. Begin second ordered list element. Ipsum faucibus vitae aliquet nec.
 *    Ligula ullamcorper malesuada proin libero. End second ordered list
 *    element.
 * 3. Begin third ordered list element. Dictum sit amet justo donec enim.
 *    Pharetra convallis posuere morbi leo urna molestie. End third ordered
 *    list element.
 *
 * Senectus et netus et malesuada fames ac. Tincidunt lobortis feugiat vivamus
 * at augue eget arcu dictum varius.
 */
struct MidLevelSymbol {};

}  // namespace drake
