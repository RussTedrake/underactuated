# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@bazel_skylib//lib:shell.bzl", "shell")

def _yapf_test_impl(ctx):
    args = []
    files = ctx.files.srcs

    if ctx.attr.config and ctx.attr.style:
        fail("Arguments config and style are mutually exclusive")

    if ctx.file.config != None:
        args.extend(["--style", shell.quote(ctx.file.config.short_path)])
        files = files + [ctx.file.config]

    if ctx.attr.parallel:
        args.append("--parallel")

    if ctx.attr.style:
        args.extend(["--style", ctx.attr.style])

    if ctx.attr.verbose:
        args.append("--verbose")

    yapf = shell.quote(
        ctx.attr._yapf[DefaultInfo].files_to_run.executable.short_path,
    )

    run_srcs = [paths.join(
        "${BUILD_WORKSPACE_DIRECTORY}",
        f.short_path,
    ) for f in ctx.files.srcs]

    test_srcs = [shell.quote(f.short_path) for f in ctx.files.srcs]

    substitutions = {
        "@@ARGS@@": " ".join(args),
        "@@GENERATED_FILE_MARKER@@": "GENERATED FILE DO NOT EDIT",
        "@@LABEL@@": str(ctx.label),
        "@@RUN_SRCS@@": " ".join(run_srcs),
        "@@TEST_SRCS@@": " ".join(test_srcs),
        "@@YAPF@@": yapf,
    }

    ctx.actions.expand_template(
        template = ctx.file._template,
        output = ctx.outputs.executable,
        substitutions = substitutions,
        is_executable = True,
    )

    runfiles = ctx.runfiles(files = files).merge(
        ctx.attr._yapf[DefaultInfo].default_runfiles,
    )

    return [DefaultInfo(runfiles = runfiles)]

_PY_YAPF_TEST_ATTRS = {
    "srcs": attr.label_list(
        doc = "The list of source files that are to be processed",
        allow_empty = False,
        allow_files = [".py"],
    ),
    "config": attr.label(
        allow_single_file = [".cfg", ".yapf"],
        doc = "The name of a file with style settings. This must be a " +
              "'setup.cfg' file with a `[yapf]` section or a '.style.yapf' " +
              "file with a `[style]` section (mutually exclusive with " +
              "`style`).",
    ),
    "parallel": attr.bool(
        doc = "Run yapf in parallel when formatting multiple files",
    ),
    "style": attr.string(
        doc = "The style name (mutually exclusive with `config`)",
        values = ["", "chromium", "facebook", "google", "pep8"],
    ),
    "verbose": attr.bool(doc = "Print out file names while processing"),
    "_template": attr.label(
        default = "//tools/rt/yapf:yapf_runner.bash.in",
        doc = "The name of the yapf test runner template",
        allow_single_file = True,
    ),
    "_yapf": attr.label(
        default = "//tools/rt/yapf",
        doc = "The name of the yapf console script",
        cfg = "host",
    ),
}  # buildifier: disable=unsorted-dict-items

_py_yapf_test = rule(
    implementation = _yapf_test_impl,
    attrs = _PY_YAPF_TEST_ATTRS,
    doc = "Check formatting of Python code",
    test = True,
    provides = [DefaultInfo],
)

def py_yapf_test(**kwargs):
    """Format Python code.

    Args:
        name: A unique name for this target.
        srcs: he list of source (".py") files that are to be processed.
        config The name of a file with style settings. This must be a
            "setup.cfg" file with a `[yapf]` section or a ".style.yapf" file
            with a `[style]` section.
        diff: Print the diff for the fixed source.
        parallel: Run yapf in parallel when formatting multiple files.
        style: The style name (if specified, must be `chromium`, `facebook`,
           `google`, or  `pep8`).
        verbose: Print out file names while processing"),
    """
    size = kwargs.get("size", "small")
    kwargs["size"] = size
    tags = kwargs.get("tags", [])
    if "block-network" not in tags:
        tags.append("block-network")
    if "require-network" in tags:
        tags.remove("require-network")
    kwargs["tags"] = tags
    _py_yapf_test(**kwargs)
