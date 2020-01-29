# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@rules_python//python:pip.bzl", "pip3_import", "pip_repositories")
load("@rules_python//python:repositories.bzl", "py_repositories")

def _pip3_import_attrs(attrs):
    if "requirements" not in attrs or attrs["requirements"] == None:
        attrs["requirements"] = "//:requirements.txt"
    return attrs

def rt_python_repositories():
    py_repositories()
    pip_repositories()

def rt_pip3_import(**attrs):
    return pip3_import(**_pip3_import_attrs(attrs))
