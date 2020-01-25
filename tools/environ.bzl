# -*- mode: python -*-
# vi: set ft=python :

# Copyright (c) 2018, Toyota Research Institute.
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

# Write out a repository that contains:
# - An empty BUILD file, to define a package.
# - An environ.bzl file with variable assignments for each ENV_NAMES item.
def _impl(repository_ctx):
    vars = repository_ctx.attr._vars
    bzl_content = []
    for key in vars:
        value = repository_ctx.os.environ.get(key, "")
        bzl_content.append("{}='{}'\n".format(key, value))
    repository_ctx.file(
        "BUILD.bazel",
        content = "\n",
        executable = False,
    )
    repository_ctx.file(
        "environ.bzl",
        content = "".join(bzl_content),
        executable = False,
    )

def environ_repository(name = None, vars = []):
    """Provide specific environment variables for use in a WORKSPACE file.
    The `vars` are the environment variables to provide.

    Example:
        environ_repository(name = "foo", vars = ["BAR", "BAZ"])
        load("@foo//:environ.bzl", "BAR", "BAZ")
        print(BAR)
    """
    rule = repository_rule(
        implementation = _impl,
        attrs = {
            "_vars": attr.string_list(default = vars),
        },
        local = True,
        environ = vars,
    )
    rule(name = name)
