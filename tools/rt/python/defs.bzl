# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@pip//:requirements.bzl", "all_requirements")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("//tools/rt/pycodestyle:def.bzl", "py_pycodestyle_test")
load("//tools/rt/pydocstyle:def.bzl", "py_pydocstyle_test")
load("//tools/rt/yapf:defs.bzl", "py_yapf_test")

def _common_attrs(attrs):
    if "deps" in attrs and attrs["deps"] != None:
        attrs["deps"] = attrs["deps"] + ["@drake//bindings/pydrake"] + all_requirements
    else:
        attrs["deps"] = ["@drake//bindings/pydrake"] + all_requirements
    attrs["srcs_version"] = "PY3"
    if "tags" in attrs and attrs["tags"] != None:
        if ("block-network" not in attrs["tags"] and
            "requires-network" not in attrs["tags"]):
            attrs["tags"] = attrs["tags"] + ["block-network"]
    else:
        attrs["tags"] = ["block-network"]
    return attrs

def _binary_attrs(attrs):
    attrs["legacy_create_init"] = False
    attrs["python_version"] = "PY3"
    return _common_attrs(attrs)

def _style_test_attrs(attrs):
    if "config" not in attrs or attrs["config"] == None:
        attrs["config"] = "//:setup.cfg"
    if "size" not in attrs or attrs["size"] == None:
        attrs["size"] = "small"
    return attrs

def _test_attrs(attrs):
    if "size" not in attrs or attrs["size"] == None:
        attrs["size"] = "small"
    return _binary_attrs(attrs)

def _pycodestyle_test_attrs(attrs):
    if "tags" in attrs and attrs["tags"] != None:
        attrs["tags"] = attrs["tags"] + ["pycodestyle"]
    else:
        attrs["tags"] = ["pycodestyle"]
    return _style_test_attrs(attrs)

def _pydocstyle_test_attrs(attrs):
    if "tags" in attrs and attrs["tags"] != None:
        attrs["tags"] = attrs["tags"] + ["pydocstyle"]
    else:
        attrs["tags"] = ["pydocstyle"]
    return _style_test_attrs(attrs)

def _yapf_test_attrs(attrs):
    if "tags" in attrs and attrs["tags"] != None:
        attrs["tags"] = attrs["tags"] + ["yapf"]
    else:
        attrs["tags"] = ["yapf"]
    return _style_test_attrs(attrs)

def rt_py_binary(**attrs):
    if attrs.pop("pycodestyle", True):
        rt_py_pycodestyle_test(
            name = attrs["name"] + "_pycodestyle",
            srcs = attrs["srcs"],
        )
    if attrs.pop("pydocstyle", True):
        rt_py_pydocstyle_test(
            name = attrs["name"] + "_pydocstyle",
            srcs = attrs["srcs"],
        )
    if attrs.pop("yapf", True):
        rt_py_yapf_test(
            name = attrs["name"] + "_yapf",
            srcs = attrs["srcs"],
        )
    py_binary(**_binary_attrs(attrs))

def rt_py_library(**attrs):
    if attrs.pop("pycodestyle", True):
        rt_py_pycodestyle_test(
            name = attrs["name"] + "_pycodestyle",
            srcs = attrs["srcs"],
        )
    if attrs.pop("pydocstyle", True):
        rt_py_pydocstyle_test(
            name = attrs["name"] + "_pydocstyle",
            srcs = attrs["srcs"],
        )
    if attrs.pop("yapf", True):
        rt_py_yapf_test(
            name = attrs["name"] + "_yapf",
            srcs = attrs["srcs"],
        )
    py_library(**_common_attrs(attrs))

def rt_py_pycodestyle_test(**attrs):
    py_pycodestyle_test(**_pycodestyle_test_attrs(attrs))

def rt_py_pydocstyle_test(**attrs):
    py_pydocstyle_test(**_pydocstyle_test_attrs(attrs))

def rt_py_yapf_test(**attrs):
    py_yapf_test(**_yapf_test_attrs(attrs))

def rt_py_test(**attrs):
    if attrs.pop("pycodestyle", True):
        rt_py_pycodestyle_test(
            name = attrs["name"] + "_pycodestyle",
            srcs = attrs["srcs"],
        )
    if attrs.pop("pydocstyle", True):
        rt_py_pydocstyle_test(
            name = attrs["name"] + "_pydocstyle",
            srcs = attrs["srcs"],
        )
    if attrs.pop("yapf", True):
        rt_py_yapf_test(
            name = attrs["name"] + "_yapf",
            srcs = attrs["srcs"],
        )
    py_test(**_test_attrs(attrs))
