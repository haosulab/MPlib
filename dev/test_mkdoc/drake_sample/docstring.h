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

static const char *__doc_drake_mkdoc_test_AnonymousConstant = R"doc(Anonymous enum's constant.)doc";

static const char *__doc_drake_mkdoc_test_Class =
R"doc(* Begin first unordered list element. Volutpat blandit aliquam etiam erat
  velit scelerisque. End first unordered list element.
* Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
  Ipsum dolor sit amet consectetur adipiscing. End second unordered list
  element.
* Begin third unordered list element. Hac habitasse platea dictumst quisque
  sagittis purus sit. End third unordered list element.)doc";

static const char *__doc_drake_mkdoc_test_Class_Class =
R"doc(
Custom constructor 1.)doc";

static const char *__doc_drake_mkdoc_test_Class_Class_2 =
R"doc(
Custom constructor 2. Ut tristique et egestas quis ipsum suspendisse ultrices
gravida. Suscipit tellus mauris a diam. Maecenas accumsan lacus vel facilisis
volutpat est.

Ut consequat semper viverra nam libero.)doc";

static const char *__doc_drake_mkdoc_test_Class_Class_3 =
R"doc(
Custom constructor 3. Integer quis auctor elit sed vulputate mi sit.

Custom constructor 3. If the object is currently attached, disallow collision
between the object and previous touch_links. Updates acm_ to allow collisions
between attached object and touch_links.

:param name: normal object name to attach
:param art_name: name of the planned articulation to attach to
:param link_id: index of the link of the planned articulation to attach to
:param pose: attached pose (relative pose from attached link to object)
:param touch_links: link names that the attached object touches
:raises ValueError: if normal object with given name does not exist or if
    planned articulation with given name does not exist)doc";

static const char *__doc_drake_mkdoc_test_Class_Nested =
R"doc(Protected nested class. Sed turpis tincidunt id aliquet. Egestas sed sed risus
pretium.)doc";

static const char *__doc_drake_mkdoc_test_Class_ProtectedMethod =
R"doc(
Protected method. Nibh sed pulvinar proin gravida hendrerit. Orci phasellus
egestas tellus rutrum tellus pellentesque eu.)doc";

static const char *__doc_drake_mkdoc_test_Class_PublicMethod =
R"doc(
)doc";

static const char *__doc_drake_mkdoc_test_Class_PublicStatic =
R"doc(
Sed faucibus turpis in eu mi bibendum neque egestas. @pre Begin precondition.
Cras fermentum odio eu feugiat pretium nibh. Ornare suspendisse sed nisi lacus
sed viverra tellus. End precondition. @post Begin postcondition. Tortor id
aliquet lectus proin nibh nisl condimentum id. End postcondition.)doc";

static const char *__doc_drake_mkdoc_test_Class_PublicTemplateMethod =
R"doc(
)doc";

static const char *__doc_drake_mkdoc_test_Class_do_stuff =
R"doc(
Docstring 1. @pydrake_mkdoc_identifier{stuff_1})doc";

static const char *__doc_drake_mkdoc_test_Class_do_stuff_2 =
R"doc(
Docstring 2. @pydrake_mkdoc_identifier{stuff_2})doc";

static const char *__doc_drake_mkdoc_test_Class_get_foo =
R"doc(
Overloaded only by its const-ness.)doc";

static const char *__doc_drake_mkdoc_test_Class_get_foo_2 =
R"doc(
The const one.)doc";

static const char *__doc_drake_mkdoc_test_Class_overloaded_method =
R"doc(
This one takes an int.)doc";

static const char *__doc_drake_mkdoc_test_Class_overloaded_method_2 =
R"doc(
This one takes a double.)doc";

static const char *__doc_drake_mkdoc_test_Class_overloaded_method_3 =
R"doc(
This one takes an int and a double.)doc";

static const char *__doc_drake_mkdoc_test_Class_overloaded_method_4 =
R"doc(
This one takes the road less traveled.)doc";

static const char *__doc_drake_mkdoc_test_Class_overloaded_method_5 =
R"doc(
This one takes a non-primitive type.)doc";

static const char *__doc_drake_mkdoc_test_Class_overloaded_with_same_doc =
R"doc(
Different overload with same doc.)doc";

static const char *__doc_drake_mkdoc_test_Class_overloaded_with_same_doc_2 =
R"doc(
Different overload with same doc.)doc";

static const char *__doc_drake_mkdoc_test_Class_protected_member = R"doc(Protected member, public documentation.)doc";

static const char *__doc_drake_mkdoc_test_Enum =
R"doc(Enumeration. Feugiat scelerisque varius morbi enim. Facilisis leo vel fringilla
est ullamcorper eget nulla facilisi.)doc";

static const char *__doc_drake_mkdoc_test_EnumClass =
R"doc(Enumeration class. Malesuada fames ac turpis egestas integer eget aliquet nibh
praesent.)doc";

static const char *__doc_drake_mkdoc_test_EnumClass_EnumClassConstant = R"doc(Enumeration class constant. Vestibulum mattis.)doc";

static const char *__doc_drake_mkdoc_test_Enum_EnumConstant = R"doc(Enumeration constant.)doc";

static const char *__doc_drake_mkdoc_test_Struct =
R"doc(Struct. Sed elementum tempus egestas sed sed risus pretium. Vel pharetra vel
turpis nunc. @deprecated Begin deprecated. Est pellentesque elit ullamcorper
dignissim cras tincidunt lobortis. End deprecated.)doc";

static const char *__doc_drake_mkdoc_test_Struct_Serialize =
R"doc(
See implementing_serialize "Implementing Serialize".)doc";

static const char *__doc_drake_mkdoc_test_Struct_field_1 =
R"doc(Field 1. Sit amet cursus sit amet dictum sit amet. Id leo in vitae turpis massa
sed elementum tempus.)doc";

static const char *__doc_drake_mkdoc_test_Struct_field_2 = R"doc(Field 2. Consectetur libero id faucibus nisl tincidunt eget nullam non nisi.)doc";

static const char *__doc_drake_mkdoc_test_TemplateClass =
R"doc(Template class. Mauris pharetra et ultrices neque ornare aenean euismod
elementum.)doc";

static const char *__doc_drake_mkdoc_test_TemplateClass_2 =
R"doc(
)doc";

static const char *__doc_drake_mkdoc_test_TemplateClass_3 =
R"doc(
Single argument int constructor.)doc";

static const char *__doc_drake_mkdoc_test_TemplateClass_4 =
R"doc(
Scalar-converting copy constructor.)doc";

static const char *__doc_drake_mkdoc_test_TemplateClass_5 =
R"doc(Specialize. Nisl pretium fusce id velit ut tortor pretium viverra. Quis ipsum
suspendisse ultrices gravida dictum fusce ut.)doc";

static const char *__doc_drake_mkdoc_test_TemplateClass_TemplateClass =
R"doc(
Single argument int constructor.)doc";

static const char *__doc_drake_mkdoc_test_TemplateClass_TemplateClass_2 =
R"doc(
Scalar-converting copy constructor.)doc";

static const char *__doc_drake_mkdoc_test_func =
R"doc(
Function. Mi sit amet mauris commodo quis.)doc";

static const char *__doc_drake_mkdoc_test_func_2 =
R"doc(
Function, overload 1. Velit ut tortor pretium viverra suspendisse potenti nullam
ac tortor.)doc";

static const char *__doc_drake_mkdoc_test_func_3 =
R"doc(
Function, template overload. Pellentesque diam volutpat commodo sed egestas
egestas fringilla phasellus faucibus.)doc";

static const char *__doc_namespace_1_MySimpleSystem =
R"doc(My simple system.

@system name: Wooh input_ports: - u output_ports: - y @endsystem)doc";

static const char *__doc_namespace_1_namespace_2_DummyClass = R"doc()doc";

static const char *__doc_namespace_1_namespace_2_DummyClass_DummyClass =
R"doc(
)doc";

static const char *__doc_namespace_1_namespace_2_DummyClass_DummyClass_2 =
R"doc(
)doc";

static const char *__doc_namespace_1_namespace_2_DummyClass_DummyClass_3 =
R"doc(
)doc";

static const char *__doc_namespace_1_namespace_2_DummyClass_static_function_1 =
R"doc(
)doc";

static const char *__doc_namespace_1_namespace_2_DummyClass_static_function_2 =
R"doc(
)doc";

static const char *__doc_namespace_1_namespace_2_DummyClass_struct_2 =
R"doc(
)doc";

static const char *__doc_namespace_1_namespace_2_Struct1 =
R"doc(Quam odio at est.

Proin eleifend nisi et nibh. Maecenas a lacus. Mauris porta quam non massa
molestie scelerisque. Nulla sed ante at lorem suscipit rutrum. Nam quis tellus.
Cras elit nisi, ornare a, condimentum vitae, rutrum sit amet, tellus. Maecenas)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_1 = R"doc(Et, ornare sagittis, tellus. Fusce felis.)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_2 = R"doc(Nulla a augue. Pellentesque sed est.)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_3 = R"doc(Imperdiet tristique, interdum a, dolor.)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_4 = R"doc(Tempor lobortis turpis. Sed tellus velit, ullamcorper.)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_5 = R"doc(Id, rutrum auctor, ullamcorper sed, orci. In.)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_6 = R"doc(Fames ac turpis egestas. Sed vitae eros. Nulla.)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_7 = R"doc(Condimentum. Donec arcu quam, dictum accumsan, convallis.)doc";

static const char *__doc_namespace_1_namespace_2_Struct1_var_8 = R"doc(Volutpat. Donec non tortor. Vivamus posuere nisi mollis.)doc";

static const char *__doc_namespace_1_namespace_2_Struct2 = R"doc()doc";

static const char *__doc_namespace_1_namespace_2_Struct3 = R"doc()doc";

static const char *__doc_namespace_1_namespace_2_Struct4 = R"doc()doc";

static const char *__doc_namespace_1_namespace_2_Struct5 = R"doc()doc";

static const char *__doc_namespace_1_namespace_2_Struct6 = R"doc()doc";

/* ----- Begin of custom docstring section ----- */

/* ----- End of custom docstring section ----- */

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
