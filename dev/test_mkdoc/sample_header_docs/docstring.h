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

static const char *__doc_RootLevelSymbol = R"doc(Root-level symbol. Magna fermentum iaculis eu non diam phasellus vestibulum.)doc";

static const char *__doc_drake_MidLevelSymbol =
R"doc(1. Begin first ordered list element. Rutrum quisque non tellus orci ac auctor.
End first ordered list element. 2. Begin second ordered list element. Ipsum
faucibus vitae aliquet nec. Ligula ullamcorper malesuada proin libero. End
second ordered list element. 3. Begin third ordered list element. Dictum sit
amet justo donec enim. Pharetra convallis posuere morbi leo urna molestie. End
third ordered list element.

Senectus et netus et malesuada fames ac. Tincidunt lobortis feugiat vivamus at
augue eget arcu dictum varius.)doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
