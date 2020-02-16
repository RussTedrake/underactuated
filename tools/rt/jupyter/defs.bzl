# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("//tools/rt/python:defs.bzl", "rt_py_binary", "rt_py_test")

def _nbconvert(attrs, testonly = False):
    out = "{}.ipynb.py".format(attrs["name"])
    native.genrule(
        name = "{}_nbconvert".format(attrs["name"]),
        testonly = testonly,
        srcs = attrs["srcs"],
        outs = [out],
        cmd = "$(location //tools/rt/jupyter:nbconvert) $< > $@",
        tools = ["//tools/rt/jupyter:nbconvert"],
        visibility = ["//visibility:private"],
    )
    attrs["main"] = out
    attrs["srcs"] = [out]
    return _common_attrs(attrs)

def _common_attrs(attrs):
    # TODO(jamiesnape): Enable pycodestyle for ipynb.
    attrs["pycodestyle"] = False
    attrs["pydocstyle"] = False
    attrs["yapf"] = False
    if "tags" in attrs and attrs["tags"] != None:
        attrs["tags"] = attrs["tags"] + ["ipynb"]
    else:
        attrs["tags"] = ["ipynb"]
    return attrs

def rt_ipynb_binary(**attrs):
    rt_py_binary(**_nbconvert(attrs, testonly = attrs.get("testonly", False)))

def rt_ipynb_test(**attrs):
    if "size" not in attrs or attrs["size"] == None:
        attrs["size"] = "medium"
    if "timeout" not in attrs or attrs["timeout"] == None:
        attrs["timeout"] = "short"
    rt_py_test(**_nbconvert(attrs, testonly = attrs.get("testonly", True)))
