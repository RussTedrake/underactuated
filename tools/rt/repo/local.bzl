# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@bazel_skylib//lib:paths.bzl", "paths")

_LOCAL_FILE_BUILD_BAZEL = """# -*- mode: python -*-
# vi: set ft=python :

# {}

filegroup(
    name = "file",
    srcs = ["{}"],
    visibility = ["//visibility:public"],
)
"""

_LOCAL_FILE_WORKSPACE_BAZEL = """# -*- mode: python -*-
# vi: set ft=python :

# {}

workspace(name = "{}")
"""

def _local_file_impl(repository_ctx):
    if not paths.is_absolute(repository_ctx.attr.path):
        fail("path attribute must be an absolute path")

    disallowed_files = [
        repository_ctx.path("."),
        repository_ctx.path("BUILD.bazel"),
        repository_ctx.path("BUILD"),
        repository_ctx.path("WORKSPACE.bazel"),
        repository_ctx.path("WORKSPACE"),
        repository_ctx.path("file/BUILD.bazel"),
        repository_ctx.path("file/BUILD"),
    ]

    symlinked_path = paths.join("file", repository_ctx.attr.symlinked_file_path)

    if repository_ctx.path(symlinked_path) in disallowed_files:
        fail("symlinked file path must NOT be a BUILD or WORKSPACE file")

    repository_ctx.symlink(repository_ctx.attr.path, symlinked_path)

    repository_ctx.file(
        "file/BUILD.bazel",
        content = _LOCAL_FILE_BUILD_BAZEL.format(
            "GENERATED FILE DO NOT EDIT",
            repository_ctx.attr.symlinked_file_path,
        ),
        executable = False,
    )

    repository_ctx.file(
        "WORKSPACE.bazel",
        content = _LOCAL_FILE_WORKSPACE_BAZEL.format(
            "GENERATED FILE DO NOT EDIT",
            repository_ctx.name,
        ),
        executable = False,
    )

_LOCAL_FILE_ATTRS = {
    "path": attr.string(mandatory = True),
    "symlinked_file_path": attr.string(default = "symlinked"),
}

local_file = repository_rule(
    implementation = _local_file_impl,
    attrs = _LOCAL_FILE_ATTRS,
    local = True,
)
