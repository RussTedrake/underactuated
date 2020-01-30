# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@pip//:requirements.bzl", "all_requirements")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

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

def _test_attrs(attrs):
    if "size" not in attrs or attrs["size"] == None:
        attrs["size"] = "small"
    return _binary_attrs(attrs)

def rt_py_binary(**attrs):
    py_binary(**_binary_attrs(attrs))

def rt_py_library(**attrs):
    py_library(**_common_attrs(attrs))

def rt_py_test(**attrs):
    py_test(**_test_attrs(attrs))
