# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@bazel_skylib//lib:shell.bzl", "shell")

def _py_pydocstyle_test_impl(ctx):
    args = []
    files = ctx.files.srcs

    if ctx.file.config != None:
        args.append(
            "--config={}".format(shell.quote(ctx.file.config.short_path)),
        )
        files = files + [ctx.file.config]

    if ctx.attr.convention:
        args.append("--convention={}".format(ctx.attr.format))

    if ctx.attr.count:
        args.append("--count")

    if ctx.attr.debug:
        args.append("--debug")

    if ctx.attr.explain:
        args.append("--explain")

    if ctx.attr.ignore:
        ignore_add_ignore = ",".join(ctx.attr.ignore)
        if ctx.file.config != None or ctx.attr.convention:
            args.append("--add-ignore={}".format(ignore_add_ignore))
        else:
            args.append("--ignore={}".format(ignore_add_ignore))

    if ctx.attr.ignore_decorators:
        args.append("--ignore_decorators={}".format(ctx.attr.ignore_decorators))

    if ctx.attr.select:
        select_add_select = ",".join(ctx.attr.select)
        if ctx.attr.convention:
            args.append("--add-select={}".format(select_add_select))
        else:
            args.append("--select={}".format(select_add_select))

    if ctx.attr.source:
        args.append("--source")

    if ctx.attr.verbose:
        args.append("--verbose")

    comment = "GENERATED FILE DO NOT EDIT"
    pydocstyle = shell.quote(
        ctx.attr._pydocstyle[DefaultInfo].files_to_run.executable.short_path,
    )
    srcs = [shell.quote(f.short_path) for f in ctx.files.srcs]

    substitutions = {
        "@@ARGS@@": " ".join(args),
        "@@GENERATED_FILE_MARKER@@": "GENERATED FILE DO NOT EDIT",
        "@@PYDOCSTYLE@@": pydocstyle,
        "@@SRCS@@": " ".join(srcs),
    }

    ctx.actions.expand_template(
        template = ctx.file._template,
        output = ctx.outputs.executable,
        substitutions = substitutions,
        is_executable = True,
    )

    runfiles = ctx.runfiles(files = files).merge(
        ctx.attr._pydocstyle[DefaultInfo].default_runfiles,
    )

    return [DefaultInfo(runfiles = runfiles)]

_PY_PYDOCSTYLE_TEST_ATTRS = {
    "srcs": attr.label_list(
        doc = "The list of source files that are to be processed",
        allow_empty = False,
        allow_files = [".py"],
    ),
    "config": attr.label(
        allow_single_file = [".cfg", ".ini"],
        doc = "The name of the config file. This must be a 'setup.cfg' or " +
              "'tox.ini' file with a `[pydocstyle]` section.",
    ),
    "convention": attr.string(
        doc = "The convention from which to choose the basic list of checked " +
              "errors",
        values = ["", "google", "numpy", "pep257"],
    ),
    "count": attr.bool(
        doc = "Print the total number of errors to standard out",
    ),
    "debug": attr.bool(doc = "Print debug information"),
    "explain": attr.bool(doc = "Show an explanation of each error"),
    "ignore": attr.string_list(
        doc = "The list of errors to ignore. If a convention is specified, " +
              "these are removed from the basic list of errors for the " +
              "convention.",
    ),
    "ignore_decorators": attr.string(
        doc = "Ignore any functions or methods that are decorated by a " +
              "function with a name fitting this regular expression",
    ),
    "select": attr.string_list(
        doc = "The list of errors to check. If a convention is specified, " +
              "these are added to basic list of errors for the convention.",
    ),
    "source": attr.bool(doc = "Show the source for each error"),
    "verbose": attr.bool(doc = "Print status information"),
    "_pydocstyle": attr.label(
        default = "//tools/rt/pydocstyle",
        doc = "The name of the pydocstyle console script",
        cfg = "host",
    ),
    "_template": attr.label(
        default = "//tools/rt/pydocstyle:pydocstyle_runner.bash.in",
        doc = "The name of the pydocstyle test runner template",
        allow_single_file = True,
    ),
}

_py_pydocstyle_test = rule(
    implementation = _py_pydocstyle_test_impl,
    attrs = _PY_PYDOCSTYLE_TEST_ATTRS,
    doc = "Check compliance with Python docstring conventions, such as PEP 257",
    provides = [DefaultInfo],
    test = True,
)

def py_pydocstyle_test(**kwargs):
    """Check compliance with Python docstring conventions, such as PEP 257.

    Args:
        name: A unique name for this test target.
        srcs: The list of source (".py") files that are to be processed.
        config: The name of the config file. This must be named "setup.cfg" or
            "tox.ini" and have a `[pydocstyle]` section.
        count: Print the total number of errors to standard out.
        convention: The convention from which to choose the basic list of
            checked errors (if specified, must be `google`, `numpy`, or
            `pep257`).
        debug: Print debug information.
        explain: Show an explanation of each error.
        ignore: The list of errors to ignore. If a convention is specified,
            these are removed from the basic list of errors for the convention.
        ignore_decorators: Ignore any functions or methods that are decorated by
            a function with a name fitting this regular expression.
        select: The list of errors to check. If a convention is specified, these
            are added to basic list of errors for the convention.
        source: Show the source for each error.
        verbose: Print status information.
    """

    size = kwargs.get("size", "small")
    kwargs["size"] = size
    tags = kwargs.get("tags", [])
    if "block-network" not in tags:
        tags.append("block-network")
    if "require-network" in tags:
        tags.remove("require-network")
    kwargs["tags"] = tags
    _py_pydocstyle_test(**kwargs)
