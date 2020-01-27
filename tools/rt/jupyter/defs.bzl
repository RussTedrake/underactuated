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
    return rt_py_test(**_jupyter_nbconvert(attrs, testonly = True))
