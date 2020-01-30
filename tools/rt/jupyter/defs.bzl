# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("//tools/rt/python:defs.bzl", "rt_py_binary", "rt_py_test")

def _jupyter_nbconvert(attrs, testonly = False):
    outs = ["{}.py".format(attrs["name"])]
    native.genrule(
        name = "{}_jupyter_nbconvert".format(attrs["name"]),
        testonly = testonly,
        srcs = attrs["srcs"],
        outs = outs,
        cmd = "$(location //tools/rt/jupyter:jupyter_nbconvert) $< > $@",
        tools = ["//tools/rt/jupyter:jupyter_nbconvert"],
        visibility = ["//visibility:private"],
    )
    attrs["srcs"] = outs
    return attrs

def rt_ipynb_binary(**attrs):
    return rt_py_binary(**_jupyter_nbconvert(attrs))

def rt_ipynb_test(**attrs):
    if "size" not in attrs or attrs["size"] == None:
        attrs["size"] = "medium"
    if "timeout" not in attrs or attrs["timeout"] == None:
        attrs["timeout"] = "short"
    rt_py_test(**_jupyter_nbconvert(attrs, testonly = True))
