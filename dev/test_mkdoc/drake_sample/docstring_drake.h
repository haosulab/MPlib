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
  // Symbol: RootLevelSymbol
  struct /* RootLevelSymbol */ {
    // Source: sample_header.h:31
    const char* doc =
R"""(Root-level symbol. Magna fermentum iaculis eu non diam phasellus
vestibulum.)""";
  } RootLevelSymbol;
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::MidLevelSymbol
    struct /* MidLevelSymbol */ {
      // Source: sample_header.h:51
      const char* doc =
R"""(1. Begin first ordered list element. Rutrum quisque non tellus orci ac
auctor. End first ordered list element. 2. Begin second ordered list
element. Ipsum faucibus vitae aliquet nec. Ligula ullamcorper
malesuada proin libero. End second ordered list element. 3. Begin
third ordered list element. Dictum sit amet justo donec enim. Pharetra
convallis posuere morbi leo urna molestie. End third ordered list
element.

Senectus et netus et malesuada fames ac. Tincidunt lobortis feugiat
vivamus at augue eget arcu dictum varius.)""";
    } MidLevelSymbol;
    // Symbol: drake::mkdoc_test
    struct /* mkdoc_test */ {
      // Symbol: drake::mkdoc_test::AnonymousConstant
      struct /* AnonymousConstant */ {
        // Source: sample_header.h:273
        const char* doc = R"""(Anonymous enum's constant.)""";
      } AnonymousConstant;
      // Symbol: drake::mkdoc_test::Class
      struct /* Class */ {
        // Source: sample_header.h:75
        const char* doc =
R"""(* Begin first unordered list element. Volutpat blandit aliquam etiam erat
  velit scelerisque. End first unordered list element.
* Begin second unordered list element. Eget est lorem ipsum dolor sit amet.
  Ipsum dolor sit amet consectetur adipiscing. End second unordered list
  element.
* Begin third unordered list element. Hac habitasse platea dictumst quisque
  sagittis purus sit. End third unordered list element.)""";
        // Symbol: drake::mkdoc_test::Class::Class
        struct /* ctor */ {
          // Source: sample_header.h:97
          const char* doc_0args = R"""(Custom constructor 1.)""";
          // Source: sample_header.h:104
          const char* doc_1args =
R"""(Custom constructor 2. Ut tristique et egestas quis ipsum suspendisse
ultrices gravida. Suscipit tellus mauris a diam. Maecenas accumsan
lacus vel facilisis volutpat est.

Ut consequat semper viverra nam libero.)""";
          // Source: sample_header.h:125
          const char* doc_2args =
R"""(Custom constructor 3. Integer quis auctor elit sed vulputate mi sit.

Custom constructor 3. If the object is currently attached, disallow
collision between the object and previous touch_links. Updates acm_ to
allow collisions between attached object and touch_links.

Parameter ``name:``:
    normal object name to attach

Parameter ``art_name:``:
    name of the planned articulation to attach to

Parameter ``link_id:``:
    index of the link of the planned articulation to attach to

Parameter ``pose:``:
    attached pose (relative pose from attached link to object)

Parameter ``touch_links:``:
    link names that the attached object touches

Raises:
    ValueError if normal object with given name does not exist or if
    planned articulation with given name does not exist)""";
        } ctor;
        // Symbol: drake::mkdoc_test::Class::Nested
        struct /* Nested */ {
          // Source: sample_header.h:188
          const char* doc =
R"""(Protected nested class. Sed turpis tincidunt id aliquet. Egestas sed
sed risus pretium.)""";
        } Nested;
        // Symbol: drake::mkdoc_test::Class::ProtectedMethod
        struct /* ProtectedMethod */ {
          // Source: sample_header.h:182
          const char* doc =
R"""(Protected method. Nibh sed pulvinar proin gravida hendrerit. Orci
phasellus egestas tellus rutrum tellus pellentesque eu.)""";
        } ProtectedMethod;
        // Symbol: drake::mkdoc_test::Class::PublicMethod
        struct /* PublicMethod */ {
          // Source: sample_header.h:130
          const char* doc = R"""()""";
        } PublicMethod;
        // Symbol: drake::mkdoc_test::Class::PublicStatic
        struct /* PublicStatic */ {
          // Source: sample_header.h:141
          const char* doc =
R"""(Sed faucibus turpis in eu mi bibendum neque egestas.

Precondition:
    Begin precondition. Cras fermentum odio eu feugiat pretium nibh.
    Ornare suspendisse sed nisi lacus sed viverra tellus. End
    precondition.

Postcondition:
    Begin postcondition. Tortor id aliquet lectus proin nibh nisl
    condimentum id. End postcondition.)""";
        } PublicStatic;
        // Symbol: drake::mkdoc_test::Class::PublicTemplateMethod
        struct /* PublicTemplateMethod */ {
          // Source: sample_header.h:134
          const char* doc = R"""()""";
        } PublicTemplateMethod;
        // Symbol: drake::mkdoc_test::Class::TypedefAlias
        struct /* TypedefAlias */ {
          // Source: sample_header.h:89
          const char* doc =
R"""(Typedef alias. Risus nec feugiat in fermentum posuere urna nec
tincidunt praesent.)""";
        } TypedefAlias;
        // Symbol: drake::mkdoc_test::Class::UsingAlias
        struct /* UsingAlias */ {
          // Source: sample_header.h:84
          const char* doc =
R"""(Using alias. Sit amet nisl purus in mollis nunc sed id semper.)""";
        } UsingAlias;
        // Symbol: drake::mkdoc_test::Class::do_stuff
        struct /* do_stuff */ {
          // Source: sample_header.h:172
          const char* doc_stuff_1 = R"""(Docstring 1.)""";
          // Source: sample_header.h:177
          const char* doc_stuff_2 = R"""(Docstring 2.)""";
        } do_stuff;
        // Symbol: drake::mkdoc_test::Class::get_foo
        struct /* get_foo */ {
          // Source: sample_header.h:165
          const char* doc_0args_nonconst = R"""(Overloaded only by its const-ness.)""";
          // Source: sample_header.h:168
          const char* doc_0args_const = R"""(The const one.)""";
        } get_foo;
        // Symbol: drake::mkdoc_test::Class::overloaded_method
        struct /* overloaded_method */ {
          // Source: sample_header.h:144
          const char* doc_1args_alpha = R"""(This one takes an int.)""";
          // Source: sample_header.h:147
          const char* doc_1args_bravo = R"""(This one takes a double.)""";
          // Source: sample_header.h:150
          const char* doc_2args_charlie_delta = R"""(This one takes an int and a double.)""";
          // Source: sample_header.h:153
          const char* doc_2args_double_int = R"""(This one takes the road less traveled.)""";
          // Source: sample_header.h:156
          const char* doc_1args_conststdstring = R"""(This one takes a non-primitive type.)""";
        } overloaded_method;
        // Symbol: drake::mkdoc_test::Class::overloaded_with_same_doc
        struct /* overloaded_with_same_doc */ {
          // Source: sample_header.h:159
          const char* doc = R"""(Different overload with same doc.)""";
        } overloaded_with_same_doc;
        // Symbol: drake::mkdoc_test::Class::protected_member_
        struct /* protected_member_ */ {
          // Source: sample_header.h:191
          const char* doc = R"""(Protected member, public documentation.)""";
        } protected_member_;
      } Class;
      // Symbol: drake::mkdoc_test::DrakeDeprecatedClass
      struct /* DrakeDeprecatedClass */ {
        // Source: sample_header.h:278
        const char* doc_deprecated =
R"""(I am measurably old. (Deprecated.)

Deprecated:
    Use MyNewClass instead. This will be removed from Drake on or
    after 2038-01-19.)""";
        // Symbol: drake::mkdoc_test::DrakeDeprecatedClass::a
        struct /* a */ {
          // Source: sample_header.h:291
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    a() is slow; use b() instead. This will be removed from Drake on
    or after 2038-01-19.)""";
        } a;
        // Symbol: drake::mkdoc_test::DrakeDeprecatedClass::f
        struct /* f */ {
          // Source: sample_header.h:283
          const char* doc_deprecated_1args =
R"""((Deprecated.)

Deprecated:
    f() is slow; use g() instead. Also, I like hats. This will be
    removed from Drake on or after 2038-01-19.)""";
          // Source: sample_header.h:287
          const char* doc_deprecated_0args =
R"""((Deprecated.)

Deprecated:
    f() now requires an int. This will be removed from Drake on or
    after 2038-01-19.)""";
        } f;
      } DrakeDeprecatedClass;
      // Symbol: drake::mkdoc_test::DrakeDeprecatedTemplateClass
      struct /* DrakeDeprecatedTemplateClass */ {
        // Source: sample_header.h:297
        const char* doc_deprecated =
R"""(I am symbolically old. (Deprecated.)

Deprecated:
    Templates rule! This will be removed from Drake on or after
    2038-01-19.)""";
      } DrakeDeprecatedTemplateClass;
      // Symbol: drake::mkdoc_test::Enum
      struct /* Enum */ {
        // Source: sample_header.h:259
        const char* doc =
R"""(Enumeration. Feugiat scelerisque varius morbi enim. Facilisis leo vel
fringilla est ullamcorper eget nulla facilisi.)""";
        // Symbol: drake::mkdoc_test::Enum::EnumConstant
        struct /* EnumConstant */ {
          // Source: sample_header.h:261
          const char* doc = R"""(Enumeration constant.)""";
        } EnumConstant;
      } Enum;
      // Symbol: drake::mkdoc_test::EnumClass
      struct /* EnumClass */ {
        // Source: sample_header.h:266
        const char* doc =
R"""(Enumeration class. Malesuada fames ac turpis egestas integer eget
aliquet nibh praesent.)""";
        // Symbol: drake::mkdoc_test::EnumClass::EnumClassConstant
        struct /* EnumClassConstant */ {
          // Source: sample_header.h:267
          const char* doc =
R"""(Enumeration class constant. Vestibulum mattis.)""";
        } EnumClassConstant;
      } EnumClass;
      // Symbol: drake::mkdoc_test::Struct
      struct /* Struct */ {
        // Source: sample_header.h:206
        const char* doc_deprecated =
R"""(Struct. Sed elementum tempus egestas sed sed risus pretium. Vel
pharetra vel turpis nunc.

Deprecated:
    Begin deprecated. Est pellentesque elit ullamcorper dignissim cras
    tincidunt lobortis. End deprecated.)""";
        // Symbol: drake::mkdoc_test::Struct::Serialize
        struct /* Serialize */ {
          // Source: sample_header.h:209
          const char* doc =
R"""(See implementing_serialize "Implementing Serialize".)""";
        } Serialize;
        // Symbol: drake::mkdoc_test::Struct::field_1
        struct /* field_1 */ {
          // Source: sample_header.h:212
          const char* doc =
R"""(Field 1. Sit amet cursus sit amet dictum sit amet. Id leo in vitae
turpis massa sed elementum tempus.)""";
        } field_1;
        // Symbol: drake::mkdoc_test::Struct::field_2
        struct /* field_2 */ {
          // Source: sample_header.h:215
          const char* doc =
R"""(Field 2. Consectetur libero id faucibus nisl tincidunt eget nullam non
nisi.)""";
        } field_2;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("field_1", field_1.doc),
            std::make_pair("field_2", field_2.doc),
          };
        }
      } Struct;
      // Symbol: drake::mkdoc_test::TemplateClass
      struct /* TemplateClass */ {
        // Source: sample_header.h:221
        const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Template class. Mauris pharetra et ultrices neque ornare aenean
euismod elementum.)""";
        // Symbol: drake::mkdoc_test::TemplateClass::TemplateClass<T>
        struct /* ctor */ {
          // Source: sample_header.h:227
          const char* doc_0args =
R"""(Default constructor. Condimentum mattis pellentesque id nibh tortor
id. Nisl rhoncus mattis rhoncus urna neque.)""";
          // Source: sample_header.h:230
          const char* doc_1args = R"""(Single argument int constructor.)""";
          // Source: sample_header.h:234
          const char* doc_copyconvert = R"""(Scalar-converting copy constructor.)""";
        } ctor;
      } TemplateClass;
      // Symbol: drake::mkdoc_test::func
      struct /* func */ {
        // Source: sample_header.h:59
        const char* doc_0args =
R"""(Function. Mi sit amet mauris commodo quis.)""";
        // Source: sample_header.h:62
        const char* doc_1args_param =
R"""(Function, overload 1. Velit ut tortor pretium viverra suspendisse
potenti nullam ac tortor.)""";
        // Source: sample_header.h:66
        const char* doc_1args_T =
R"""(Function, template overload. Pellentesque diam volutpat commodo sed
egestas egestas fringilla phasellus faucibus.)""";
      } func;
    } mkdoc_test;
  } drake;
  // Symbol: namespace_1
  struct /* namespace_1 */ {
    // Symbol: namespace_1::MySimpleSystem
    struct /* MySimpleSystem */ {
      // Source: sample_header.h:392
      const char* doc =
R"""(My simple system.

@system name: Wooh input_ports: - u output_ports: - y @endsystem)""";
    } MySimpleSystem;
    // Symbol: namespace_1::namespace_2
    struct /* namespace_2 */ {
      // Symbol: namespace_1::namespace_2::DummyClass
      struct /* DummyClass */ {
        // Source: sample_header.h:343
        const char* doc = R"""()""";
        // Symbol: namespace_1::namespace_2::DummyClass::Details
        struct /* Details */ {
          // Source: sample_header.h:348
          const char* doc =
R"""(Ligula. Nunc turpis. Mauris vitae sapien. Nunc.)""";
        } Details;
        // Symbol: namespace_1::namespace_2::DummyClass::DummyClass
        struct /* ctor */ {
          // Source: sample_header.h:345
          const char* doc = R"""()""";
        } ctor;
        // Symbol: namespace_1::namespace_2::DummyClass::static_function_1
        struct /* static_function_1 */ {
          // Source: sample_header.h:356
          const char* doc = R"""()""";
        } static_function_1;
        // Symbol: namespace_1::namespace_2::DummyClass::static_function_2
        struct /* static_function_2 */ {
          // Source: sample_header.h:357
          const char* doc = R"""()""";
        } static_function_2;
        // Symbol: namespace_1::namespace_2::DummyClass::struct_2
        struct /* struct_2 */ {
          // Source: sample_header.h:355
          const char* doc = R"""()""";
        } struct_2;
      } DummyClass;
      // Symbol: namespace_1::namespace_2::Struct1
      struct /* Struct1 */ {
        // Source: sample_header.h:318
        const char* doc =
R"""(Quam odio at est.

Proin eleifend nisi et nibh. Maecenas a lacus. Mauris porta quam non
massa molestie scelerisque. Nulla sed ante at lorem suscipit rutrum.
Nam quis tellus. Cras elit nisi, ornare a, condimentum vitae, rutrum
sit amet, tellus. Maecenas)""";
        // Symbol: namespace_1::namespace_2::Struct1::var_1
        struct /* var_1 */ {
          // Source: sample_header.h:320
          const char* doc =
R"""(Et, ornare sagittis, tellus. Fusce felis.)""";
        } var_1;
        // Symbol: namespace_1::namespace_2::Struct1::var_2
        struct /* var_2 */ {
          // Source: sample_header.h:322
          const char* doc = R"""(Nulla a augue. Pellentesque sed est.)""";
        } var_2;
        // Symbol: namespace_1::namespace_2::Struct1::var_3
        struct /* var_3 */ {
          // Source: sample_header.h:324
          const char* doc = R"""(Imperdiet tristique, interdum a, dolor.)""";
        } var_3;
        // Symbol: namespace_1::namespace_2::Struct1::var_4
        struct /* var_4 */ {
          // Source: sample_header.h:326
          const char* doc =
R"""(Tempor lobortis turpis. Sed tellus velit, ullamcorper.)""";
        } var_4;
        // Symbol: namespace_1::namespace_2::Struct1::var_5
        struct /* var_5 */ {
          // Source: sample_header.h:328
          const char* doc =
R"""(Id, rutrum auctor, ullamcorper sed, orci. In.)""";
        } var_5;
        // Symbol: namespace_1::namespace_2::Struct1::var_6
        struct /* var_6 */ {
          // Source: sample_header.h:330
          const char* doc =
R"""(Fames ac turpis egestas. Sed vitae eros. Nulla.)""";
        } var_6;
        // Symbol: namespace_1::namespace_2::Struct1::var_7
        struct /* var_7 */ {
          // Source: sample_header.h:332
          const char* doc =
R"""(Condimentum. Donec arcu quam, dictum accumsan, convallis.)""";
        } var_7;
        // Symbol: namespace_1::namespace_2::Struct1::var_8
        struct /* var_8 */ {
          // Source: sample_header.h:334
          const char* doc =
R"""(Volutpat. Donec non tortor. Vivamus posuere nisi mollis.)""";
        } var_8;
      } Struct1;
      // Symbol: namespace_1::namespace_2::Struct2
      struct /* Struct2 */ {
        // Source: sample_header.h:337
        const char* doc = R"""()""";
      } Struct2;
      // Symbol: namespace_1::namespace_2::Struct3
      struct /* Struct3 */ {
        // Source: sample_header.h:338
        const char* doc = R"""()""";
      } Struct3;
      // Symbol: namespace_1::namespace_2::Struct4
      struct /* Struct4 */ {
        // Source: sample_header.h:339
        const char* doc = R"""()""";
      } Struct4;
      // Symbol: namespace_1::namespace_2::Struct5
      struct /* Struct5 */ {
        // Source: sample_header.h:340
        const char* doc = R"""()""";
      } Struct5;
      // Symbol: namespace_1::namespace_2::Struct6
      struct /* Struct6 */ {
        // Source: sample_header.h:341
        const char* doc = R"""()""";
      } Struct6;
    } namespace_2;
  } namespace_1;
} mkdoc_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
