# -*- mode: python -*-
# vi: set ft=python :

# Copyright (c) 2020, Massachusetts Institute of Technology.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

load("@pip//:requirements.bzl", "all_requirements")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

def _common_attrs(attrs):
    if "deps" in attrs and attrs["deps"] != None:
        attrs["deps"] = attrs["deps"] + ["@drake//bindings/pydrake"] + all_requirements
    else:
        attrs["deps"] = ["@drake//bindings/pydrake"] + all_requirements
    attrs["srcs_version"] = "PY3"
    if "tags" in attrs and attrs["tags"] != None:
        if "requires-network" not in attrs["tags"]:
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
    return py_binary(**_binary_attrs(attrs))

def rt_py_library(**attrs):
    return py_library(**_common_attrs(attrs))

def rt_py_test(**attrs):
    return py_test(**_test_attrs(attrs))
