#!/usr/bin/env python3
"""
pybind11_mkdoc: Extract documentation from C++ header files to use it in Python bindings

Requires: libclang

Derived from https://github.com/pybind/pybind11_mkdoc and
https://github.com/RobotLocomotion/drake/blob/849d537302191f0be98875da359580d341836869/tools/workspace/pybind11/mkdoc.py#L642

  Copyright (c) 2016 Wenzel Jakob <wenzel.jakob@epfl.ch>,
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

Syntax: mkdoc.py [-o <file>] [-I<dir> ..] [-D<macro>=<value> ..] [.. header files ..]
"""

from __future__ import annotations

import argparse
import ctypes.util
import os
import platform
import re
import shlex
import sys
import textwrap
from collections import OrderedDict
from glob import glob
from multiprocessing import cpu_count
from pathlib import Path
from threading import Semaphore, Thread
from typing import Optional

from clang import cindex
from clang.cindex import AccessSpecifier, CursorKind

__version__ = "2.6.3"

CLASS_KINDS = [
    CursorKind.CLASS_DECL,
    CursorKind.STRUCT_DECL,
    CursorKind.CLASS_TEMPLATE,
]

FUNCTION_KINDS = [
    CursorKind.FUNCTION_DECL,
    CursorKind.FUNCTION_TEMPLATE,
    CursorKind.CONVERSION_FUNCTION,
    CursorKind.CXX_METHOD,
    CursorKind.CONSTRUCTOR,
]


RECURSE_LIST = [
    CursorKind.TRANSLATION_UNIT,
    CursorKind.NAMESPACE,
    CursorKind.CLASS_DECL,
    CursorKind.STRUCT_DECL,
    CursorKind.ENUM_DECL,
    CursorKind.CLASS_TEMPLATE,
]

PRINT_LIST = (
    CLASS_KINDS
    + FUNCTION_KINDS
    + [
        CursorKind.ENUM_DECL,
        CursorKind.ENUM_CONSTANT_DECL,
        CursorKind.FIELD_DECL,
        # NOTE: drake also processes type alias and typedef
        # CursorKind.TYPE_ALIAS_DECL,  # using x = y
        # CursorKind.TYPEDEF_DECL,
    ]
)

PREFIX_BLACKLIST = [CursorKind.TRANSLATION_UNIT]

CPP_OPERATORS = {
    "<=": "le",
    ">=": "ge",
    "==": "eq",
    "!=": "ne",
    "[]": "array",
    "+=": "iadd",
    "-=": "isub",
    "*=": "imul",
    "/=": "idiv",
    "%=": "imod",
    "&=": "iand",
    "|=": "ior",
    "^=": "ixor",
    "<<=": "ilshift",
    ">>=": "irshift",
    "++": "inc",
    "--": "dec",
    "<<": "lshift",
    ">>": "rshift",
    "&&": "land",
    "||": "lor",
    "!": "lnot",
    "~": "bnot",
    "&": "band",
    "|": "bor",
    "+": "add",
    "-": "sub",
    "*": "mul",
    "/": "div",
    "%": "mod",
    "<": "lt",
    ">": "gt",
    "=": "assign",
    "()": "call",
}

CPP_OPERATORS = OrderedDict(sorted(CPP_OPERATORS.items(), key=lambda t: -len(t[0])))

# 'Broadphase' culling; do not recurse inside these symbols.
SKIP_RECURSE_NAMES = [
    "Eigen",
    "detail",
    "dev",
    "google",
    "internal",
    "std",
    "tinyxml2",
]

# Filter based on access.
SKIP_ACCESS = [
    AccessSpecifier.PRIVATE,
]

job_count = cpu_count()
job_semaphore = Semaphore(job_count)
errors_detected = False
docstring_width = 80


class NoFilenamesError(ValueError):
    pass


def d(s) -> str:
    return s if isinstance(s, str) else s.decode("utf8")


def is_accepted_cursor(cursor, name_chain) -> bool:
    """
    Determines if a symbol should be visited or not, given the cursor and the
    name chain.
    """
    name = cursor.spelling
    # N.B. See TODO in `get_name_chain`.
    for piece in name_chain + (name,):
        if piece in SKIP_RECURSE_NAMES:
            return False
    if cursor.access_specifier in SKIP_ACCESS:
        return False
    # TODO(eric.cousineau): Remove `cursor.is_default_method()`? May make
    # things unstable.
    if cursor.kind in CLASS_KINDS and not cursor.is_definition():
        # Don't process forward declarations.  If we did, we'd define the class
        # overview documentation twice; both cursors have a .raw_comment value.
        return False

    # Explicit Template Instantiation Declaration
    init_token_str = " ".join([token.spelling for token in cursor.get_tokens()][:3])
    if (
        cursor.kind == CursorKind.CLASS_DECL
        and init_token_str == "extern template class"
    ) or (
        cursor.kind == CursorKind.STRUCT_DECL
        and init_token_str == "extern template struct"
    ):
        return False
    return True


def sanitize_name(name):
    name = re.sub(r"type-parameter-0-([0-9]+)", r"T\1", name)
    for k, v in CPP_OPERATORS.items():
        name = name.replace("operator%s" % k, "operator_%s" % v)
    name = re.sub("<.*>", "", name)
    name = "".join([ch if ch.isalnum() else "_" for ch in name])
    name = re.sub("_$", "", re.sub("_+", "_", name))
    return "__doc_" + name


def process_comment(comment: str) -> str:
    # Remove C++ comment syntax
    s = remove_cpp_comment_syntax(comment)

    # HTML tags. Support both lowercase and uppercase tags.
    # TODO (betsymcphail): Not tested
    s = replace_html_tags(s)

    s = s.replace("``true``", "``True``")
    s = s.replace("``false``", "``False``")
    s = s.replace("``nullptr``", "``None``")  # nullptr is None in Python
    s = s.replace("``std::nullopt``", "``None``")  # std::nullopt is None in Python

    # Exceptions
    s = replace_exceptions(s)

    # Doxygen tags
    s = process_doxygen_commands(s)

    # Reflow text where appropriate.
    s = reflow(s)
    return s


def remove_cpp_comment_syntax(comment: str) -> str:
    result = ""
    leading_spaces = float("inf")
    for line in comment.expandtabs(tabsize=4).splitlines():
        line = line.strip()
        if line.endswith("*/"):
            line = line[:-2].rstrip("*")
        if line.startswith("/*"):
            line = line[2:].lstrip("*!<")
        elif line.startswith("//"):
            line = line[2:].lstrip("/!<")
        elif line.startswith("*"):
            line = line[1:]
        if len(line) > 0:
            leading_spaces = min(leading_spaces, len(line) - len(line.lstrip()))
        result += line + "\n"

    if leading_spaces != float("inf"):
        result2 = ""
        for line in result.splitlines():
            result2 += line[leading_spaces:] + "\n"
        result = result2
    return result


def replace_html_tags(s):
    s = re.sub(r"<tt>(.*?)</tt>", r"``\1``", s, flags=re.DOTALL)
    s = re.sub(r"<pre>(.*?)</pre>", r"```\n\1\n```\n", s, flags=re.DOTALL)
    s = re.sub(r"<em>(.*?)</em>", r"*\1*", s, flags=re.DOTALL)
    s = re.sub(r"<b>(.*?)</b>", r"**\1**", s, flags=re.DOTALL)
    s = re.sub(r"[\\@]f\$(.*?)[\\@]f\$", r":math:`\1`", s, flags=re.DOTALL)
    s = re.sub(r"<li>", r"\n\n* ", s)
    s = re.sub(r"</?ul>", r"", s)
    s = re.sub(r"</li>", r"\n\n", s)
    return s


def replace_exceptions(s):
    s = s.replace("std::bad_alloc", "MemoryError")
    s = s.replace("std::bad_any_cast", "RuntimeError")
    s = s.replace("std::bad_array_new_length", "MemoryError")
    s = s.replace("std::bad_cast", "RuntimeError")
    s = s.replace("std::bad_exception", "RuntimeError")
    s = s.replace("std::bad_function_call", "RuntimeError")
    s = s.replace("std::bad_optional_access", "RuntimeError")
    s = s.replace("std::bad_typeid", "RuntimeError")
    s = s.replace("std::bad_variant_access", "RuntimeError")
    s = s.replace("std::bad_weak_ptr", "RuntimeError")
    s = s.replace("std::domain_error", "ValueError")
    s = s.replace("std::exception", "RuntimeError")
    s = s.replace("std::future_error", "RuntimeError")
    s = s.replace("std::invalid_argument", "ValueError")
    s = s.replace("std::length_error", "ValueError")
    s = s.replace("std::logic_error", "RuntimeError")
    s = s.replace("std::out_of_range", "ValueError")
    s = s.replace("std::overflow_error", "RuntimeError")
    s = s.replace("std::range_error", "ValueError")
    s = s.replace("std::regex_error", "RuntimeError")
    s = s.replace("std::runtime_error", "RuntimeError")
    s = s.replace("std::system_error", "RuntimeError")
    s = s.replace("std::underflow_error", "RuntimeError")
    return s


def process_doxygen_commands(s):
    # Doxygen tags
    cpp_group = r"([^\s]+)"
    param_group = r"([\[\w:,\]]+)"

    s = re.sub(r"[\\@][cp]\s+%s" % cpp_group, r"``\1``", s)
    s = re.sub(r"[\\@]a\s+%s" % cpp_group, r"*\1*", s)
    s = re.sub(r"[\\@]e\s+%s" % cpp_group, r"*\1*", s)
    s = re.sub(r"[\\@]em\s+%s" % cpp_group, r"*\1*", s)
    s = re.sub(r"[\\@]b\s+%s" % cpp_group, r"**\1**", s)
    s = re.sub(r"[\\@]ingroup\s+%s" % cpp_group, r"", s)
    s = re.sub(
        rf"[\\@]param{param_group}?\s+{cpp_group}:",
        # r"\n\n$Parameter ``\2``:\n\n",
        r"\n$:param \2:\n",
        s,
    )
    s = re.sub(
        rf"[\\@]tparam{param_group}?\s+{cpp_group}:",
        # r"\n\n$Template parameter ``\2``:\n\n",
        r"\n$:tparam \2:\n",
        s,
    )

    # Remove class and struct tags
    s = re.sub(r"[\\@](class|struct)\s+.*", "", s)

    # Ordering is significant for command names with a common prefix.
    for in_, out_ in (
        ("returns", "return"),
        ("return", "return"),
        ("authors", "authors"),
        ("author", "author"),
        ("copyright", "copyright"),
        ("date", "date"),
        ("note", "note"),
        ("remark", "remark"),
        ("sa", "seealso"),
        ("see", "seealso"),
        ("extends", "base"),
        ("throws", "raises"),
        ("throw", "raises"),
        ("version", "version"),
    ):
        if out_ == "raises":
            s = re.sub(rf"[\\@]{in_}\s*(\w+Error):*", rf"\n$:{out_} \1:\n", s)
        else:
            s = re.sub(
                # r"[\\@]%s\s*" % in_,
                rf"[\\@]{in_}\s*:*",
                # r"\n\n$%s:\n\n" % out_,
                rf"\n$:{out_}:\n",
                s,
            )

    s = re.sub(r"[\\@]details\s*", r"\n\n", s)
    s = re.sub(r"[\\@]brief\s*", r"", s)
    s = re.sub(r"[\\@]short\s*", r"", s)
    s = re.sub(r"[\\@]ref\s*", r"", s)

    s = re.sub(
        r"[\\@]code\s?(.*?)\s?[\\@]endcode", r"```\n\1\n```\n", s, flags=re.DOTALL
    )
    s = re.sub(
        r"[\\@]warning\s?(.*?)\s?\n\n", r"$.. warning::\n\n\1\n\n", s, flags=re.DOTALL
    )
    # Deprecated expects a version number for reST and not for Doxygen. Here the first
    # word of the doxygen directives is assumed to correspond to the version number
    s = re.sub(
        r"[\\@]deprecated\s(.*?)\s?(.*?)\s?\n\n",
        r"$.. deprecated:: \1\n\n\2\n\n",
        s,
        flags=re.DOTALL,
    )
    s = re.sub(
        r"[\\@]since\s?(.*?)\s?\n\n", r".. versionadded:: \1\n\n", s, flags=re.DOTALL
    )
    s = re.sub(r"[\\@]todo\s?(.*?)\s?\n\n", r"$.. todo::\n\n\1\n\n", s, flags=re.DOTALL)

    return s


def reflow(s):
    # Re-flow text
    wrapper = textwrap.TextWrapper()
    wrapper.break_long_words = False
    wrapper.break_on_hyphens = False
    wrapper.drop_whitespace = True
    wrapper.expand_tabs = True
    wrapper.replace_whitespace = True
    wrapper.width = docstring_width
    wrapper.initial_indent = wrapper.subsequent_indent = ""

    result = ""
    in_code_segment = False
    for x in re.split(r"(```)", s):
        if x == "```":
            if not in_code_segment:
                result += "```\n"
            else:
                result += "\n```\n\n"
            in_code_segment = not in_code_segment
        elif in_code_segment:
            result += x.strip()
        else:
            for y in re.split(r"(?: *\n *){2,}", x):
                lines = re.split(r"(?: *\n *)", y)
                # Do not reflow lists or section headings.
                if re.match(r"^(?:[*+\-]|[0-9]+[.)]) ", lines[0]) or (
                    len(lines) > 1
                    and (
                        lines[1] == "=" * len(lines[0])
                        or lines[1] == "-" * len(lines[0])
                    )
                ):
                    result += y + "\n\n"
                else:
                    y_no_linebreak = re.sub(r"\s+", " ", y).strip()
                    if len(y_no_linebreak) > 0 and y_no_linebreak[0] == "$":
                        # wrapper.initial_indent = wrapper.subsequent_indent = " " * 4
                        wrapper.subsequent_indent = " " * 4
                        y_no_linebreak = y_no_linebreak[1:]
                    else:
                        # wrapper.initial_indent = wrapper.subsequent_indent = ""
                        wrapper.subsequent_indent = ""

                    wrapped = wrapper.fill(y_no_linebreak)
                    result += wrapped + ("\n" if wrapped.startswith(":") else "\n\n")

    return result.rstrip().lstrip("\n")


def get_name_chain(cursor):
    """
    Extracts the pieces for a namespace-qualified name for a symbol.
    """
    # TODO(eric.cousineau): Try to restrict the name_chain to end with name. I
    # briefly tried this once by culling based on accepted cursors, but lost
    # needed symbols because of it.
    name = cursor.spelling
    name_chain = [name]
    p = cursor.semantic_parent
    while p and not p.kind.is_translation_unit():
        piece = p.spelling
        name_chain.insert(0, piece)
        p = p.semantic_parent
    # Prune away the names of anonymous structs and enums.
    name_chain = [x for x in name_chain if x != "" and not x.startswith("(unnamed")]
    return tuple(name_chain)


def extract(filename, cursor, prefix, output):
    if not (
        cursor.location.file is None
        or os.path.samefile(d(cursor.location.file.name), filename)
    ):
        return 0

    name_chain = get_name_chain(cursor)
    if not is_accepted_cursor(cursor, name_chain):
        return

    if cursor.kind in RECURSE_LIST:
        sub_prefix = prefix
        if cursor.kind not in PREFIX_BLACKLIST:
            if len(sub_prefix) > 0:
                sub_prefix += "_"
            sub_prefix += d(cursor.spelling)
        for i in cursor.get_children():
            extract(filename, i, sub_prefix, output)
    if cursor.kind in PRINT_LIST:
        comment = d(cursor.raw_comment) if cursor.raw_comment is not None else ""
        comment = process_comment(comment)
        # Start on a new line for function comments
        if cursor.kind in FUNCTION_KINDS:
            comment = "\n" + comment
        sub_prefix = prefix
        if len(sub_prefix) > 0:
            sub_prefix += "_"
        if len(cursor.spelling) > 0:
            name = sanitize_name(sub_prefix + d(cursor.spelling))
            output.append((name, filename, comment))


class ExtractionThread(Thread):
    def __init__(self, filename, parameters, output):
        Thread.__init__(self)
        self.filename = filename
        self.parameters = parameters
        self.output = output
        job_semaphore.acquire()

    def run(self):
        global errors_detected
        print('Processing "%s" ..' % self.filename, file=sys.stderr)
        try:
            index = cindex.Index(cindex.conf.lib.clang_createIndex(False, True))
            tu = index.parse(self.filename, self.parameters)
            extract(self.filename, tu.cursor, "", self.output)
        except BaseException:
            errors_detected = True
            raise
        finally:
            job_semaphore.release()


def read_args(args):
    parameters = []
    filenames = []
    if "-x" not in args:
        parameters.extend(["-x", "c++"])
    if not any(it.startswith("-std=") for it in args):
        parameters.append("-std=c++11")
    parameters.append("-Wno-pragma-once-outside-header")

    if platform.system() == "Darwin":
        dev_path = "/Applications/Xcode.app/Contents/Developer/"
        lib_dir = dev_path + "Toolchains/XcodeDefault.xctoolchain/usr/lib/"
        sdk_dir = dev_path + "Platforms/MacOSX.platform/Developer/SDKs"
        libclang = lib_dir + "libclang.dylib"

        if cindex.Config.library_path is None and os.path.exists(libclang):
            cindex.Config.set_library_path(os.path.dirname(libclang))

        if os.path.exists(sdk_dir):
            sysroot_dir = os.path.join(sdk_dir, next(os.walk(sdk_dir))[1][0])
            parameters.append("-isysroot")
            parameters.append(sysroot_dir)
    elif platform.system() == "Windows":
        if "LIBCLANG_PATH" in os.environ:
            library_file = os.environ["LIBCLANG_PATH"]
            if os.path.isfile(library_file):
                cindex.Config.set_library_file(library_file)
            else:
                raise FileNotFoundError(
                    "Failed to find libclang.dll! Set the LIBCLANG_PATH "
                    "environment variable to provide a path to it."
                )
        elif cindex.Config.library_path is None:
            library_file = ctypes.util.find_library("libclang.dll")
            if library_file is not None:
                cindex.Config.set_library_file(library_file)
    elif platform.system() == "Linux":
        # LLVM switched to a monolithical setup that includes everything under
        # /usr/lib/llvm{version_number}/. We glob for the library and select
        # the highest version
        def folder_version(d):
            return [int(ver) for ver in re.findall(r"(?<!lib)(?<!\d)\d+", d)]

        llvm_dir = max(
            (
                path
                for libdir in ["lib64", "lib", "lib32"]
                for path in glob("/usr/%s/llvm-*" % libdir)
                if os.path.exists(os.path.join(path, "lib", "libclang.so.1"))
            ),
            default=None,
            key=folder_version,
        )

        # Ability to override LLVM/libclang paths
        if "LLVM_DIR_PATH" in os.environ:
            llvm_dir = os.environ["LLVM_DIR_PATH"]
        elif llvm_dir is None:
            raise FileNotFoundError(
                "Failed to find a LLVM installation providing the file "
                "/usr/lib{32,64}/llvm-{VER}/lib/libclang.so.1. Make sure that "
                "you have installed the packages libclang1-{VER} and "
                "libc++-{VER}-dev, where {VER} refers to the desired "
                "Clang/LLVM version (e.g. 11). You may alternatively override "
                "the automatic search by specifying the LIBLLVM_DIR_PATH "
                "(for the LLVM base directory) and/or LIBCLANG_PATH (if "
                "libclang is located at a nonstandard location) environment "
                "variables."
            )

        if "LIBCLANG_PATH" in os.environ:
            cindex.Config.set_library_file(os.environ["LIBCLANG_PATH"])
        elif cindex.Config.library_path is None:
            cindex.Config.set_library_file(
                os.path.join(llvm_dir, "lib", "libclang.so.1")
            )
        cpp_dirs = []

        if "-stdlib=libc++" not in args:
            cpp_dirs.append(
                max(glob("/usr/include/c++/*"), default=None, key=folder_version)
            )

            cpp_dirs.append(
                max(
                    glob("/usr/include/%s-linux-gnu/c++/*" % platform.machine()),
                    default=None,
                    key=folder_version,
                )
            )
        else:
            if llvm_dir is None:
                raise FileNotFoundError(
                    "-stdlib=libc++ has been specified, but no LLVM "
                    "installation have been found on the system."
                )
            cpp_dirs.append(os.path.join(llvm_dir, "include", "c++", "v1"))

        if "CLANG_INCLUDE_DIR" in os.environ:
            clang_include_dir = os.environ["CLANG_INCLUDE_DIR"]
        else:
            clang_include_dir = max(
                glob(os.path.join(llvm_dir, "lib", "clang", "*", "include")),
                default=None,
                key=folder_version,
            )
        cpp_dirs.append(clang_include_dir)

        cpp_dirs.append("/usr/include/%s-linux-gnu" % platform.machine())
        cpp_dirs.append("/usr/include")

        # Capability to specify additional include directories manually
        if "CPP_INCLUDE_DIRS" in os.environ:
            cpp_dirs.extend([
                cpp_dir
                for cpp_dir in os.environ["CPP_INCLUDE_DIRS"].split()
                if os.path.exists(cpp_dir)
            ])

        cpp_dirs = []  # NOTE: this causes issues in quay.io/pypa/manylinux2014_x86_64
        for cpp_dir in cpp_dirs:
            if cpp_dir is None:
                continue
            parameters.extend(["-isystem", cpp_dir])
        # Include clang header files
        parameters.append(f"-I{clang_include_dir}")

    for item in args:
        if item.startswith("-"):
            parameters.append(item)
        else:
            filenames.append(item)

    if len(filenames) == 0:
        raise NoFilenamesError("args parameter did not contain any filenames")

    return parameters, filenames


def extract_all(args) -> list[str]:
    parameters, filenames = read_args(args)
    comments = []
    for filename in filenames:
        thr = ExtractionThread(filename, parameters, comments)
        thr.start()

    print("Waiting for jobs to finish ..", file=sys.stderr)
    for _ in range(job_count):
        job_semaphore.acquire()

    return comments


def write_header(comments, custom_lines: list[str], outfile=sys.stdout):
    print(
        """\
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
#endif""",  # noqa: E501
        file=outfile,
    )

    name_ctr = 1
    name_prev = None
    for name, _, comment in sorted(comments, key=lambda x: (x[0], x[1])):
        if name == name_prev:
            name_ctr += 1
            name = name + "_%i" % name_ctr
        else:
            name_prev = name
            name_ctr = 1
        print(
            '\nstatic const char *{} ={}R"doc({})doc";'.format(
                name, "\n" if "\n" in comment else " ", comment
            ),
            file=outfile,
        )

    # Custom docstring section
    print("\n" + "".join(custom_lines), end="", file=outfile)

    print(
        """
#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif""",
        file=outfile,
    )


def read_custom_docstring(outfile_path: Path) -> list[str]:
    start_line = "/* ----- Begin of custom docstring section ----- */\n"
    end_line = "/* ----- End of custom docstring section ----- */\n"

    custom_docstring_lines = []

    is_custom_line = False
    if outfile_path.is_file():
        with outfile_path.open("r") as outfile:
            for line in outfile:
                if line == start_line:
                    is_custom_line = True
                elif line == end_line:
                    is_custom_line = False
                elif is_custom_line:
                    custom_docstring_lines.append(line)
    assert not is_custom_line, "Invalid custom docstring section: no end_line provided"

    # Leave an empty section
    if len(custom_docstring_lines) == 0:
        custom_docstring_lines = ["\n"]

    return [start_line] + custom_docstring_lines + [end_line]


def mkdoc(args, width, output: Optional[str] = None):
    """
    :param args: mkdoc_args format: ["-Iinclude/", "-DDEBUG=1", "test.h", "foo.h"]
    """
    if width is not None:
        global docstring_width
        docstring_width = int(width)
    comments = extract_all(args)
    if errors_detected:
        return

    if output:
        outfile_path = Path(output).resolve()
        outfile_path.parent.mkdir(exist_ok=True)

        # Read custom docstring section (e.g., lambda functions in pybind11)
        custom_lines = read_custom_docstring(outfile_path)
        try:
            with outfile_path.open("w") as outfile:
                write_header(comments, custom_lines, outfile)
        except:
            # In the event of an error, don't leave a partially-written output file.
            outfile_path.unlink()
            raise
    else:
        write_header(comments)


def main():
    """
    Entry point for the `pybind11_mkdoc` console script.

    Parses the commandline arguments given to the console script and
    passes them on to `mkdoc`.
    """
    parser = argparse.ArgumentParser(
        prog="pybind11_mkdoc",
        description=(
            "Processes a sequence of C/C++ headers and extracts comments for "
            "use in pybind11 binding code."
        ),
        epilog="(Other compiler flags that Clang understands can also be supplied)",
        allow_abbrev=False,
    )

    parser.add_argument(
        "-V", "--version", action="version", version=f"%(prog)s {__version__}"
    )

    parser.add_argument(
        "-o",
        "--output",
        action="store",
        type=str,
        dest="output",
        metavar="<file>",
        help="Write to the specified file (default: use stdout).",
    )

    parser.add_argument(
        "-w",
        "--width",
        action="store",
        type=int,
        dest="width",
        metavar="<width>",
        help="Specify docstring width before wrapping.",
    )

    parser.add_argument(
        "-I",
        action="append",
        type=str,
        dest="include_dirs",
        metavar="<dir>",
        help="Specify an directory to add to the list of include search paths.",
    )

    parser.add_argument(
        "-D",
        action="append",
        type=str,
        metavar="<macro>=<value>",
        dest="definitions",
        help=(
            "Specify a compiler definition, i.e. define <macro> to <value> "
            "(or 1 if <value> omitted)."
        ),
    )

    parser.add_argument("header", type=str, nargs="+", help="A header file to process.")

    [parsed_args, unparsed_args] = parser.parse_known_args()

    mkdoc_args = []
    mkdoc_out = parsed_args.output
    docstring_width = parsed_args.width

    def _append_include_dir(args: list[str], include_dir: str):
        if os.path.isdir(include_dir):
            args.append(f"-I{shlex.quote(include_dir)}")
        else:
            raise FileNotFoundError(
                f"Include directoy '{shlex.quote(include_dir)}' does not exist!"
            )

    def _append_definition(args: list[str], definition: str):
        if re.search(r"^[A-Za-z_][A-Za-z0-9_]*", definition) is None:
            raise ValueError(f"Invalid macro name: {definition}")
        if "=" in definition:
            macro, value = definition.strip().split("=")
            macro = shlex.quote(macro.strip())
            value = shlex.quote(value.strip()) if value else "1"
            args.append(f"-D{macro}={value}")
        else:
            args.append(f"-D{definition}")

    # Parse "-I" include_dirs, check if it exists
    if parsed_args.include_dirs is not None:
        for include_dir in parsed_args.include_dirs:
            _append_include_dir(mkdoc_args, include_dir)

    # Parse "-D" macro definitions, check macro name is valid
    if parsed_args.definitions is not None:
        for definition in parsed_args.definitions:
            _append_definition(mkdoc_args, definition)

    # Parse additional Clang compiler flags
    for arg in unparsed_args:
        if arg.startswith("-I"):
            _append_include_dir(mkdoc_args, arg[2:].strip())
        elif arg.startswith("-D"):
            _append_definition(mkdoc_args, arg[2:].strip())
        else:
            # append argument as is and hope for the best
            mkdoc_args.append(shlex.quote(arg))

    # Prase header files
    mkdoc_args.extend(shlex.quote(header) for header in parsed_args.header)

    # mkdoc_args format: ["-Iinclude/", "-DDEBUG=1", "test.h", "foo.h"]
    mkdoc(mkdoc_args, docstring_width, mkdoc_out)


if __name__ == "__main__":
    main()
