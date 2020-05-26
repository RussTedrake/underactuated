# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@bazel_skylib//lib:shell.bzl", "shell")

def _py_pycodestyle_test_impl(ctx):
    args = []
    files = ctx.files.srcs

    if ctx.file.config != None:
        args.append(
            "--config={}".format(shell.quote(ctx.file.config.short_path)),
        )
        files = files + [ctx.file.config]

    if ctx.attr.count:
        args.append("--count")

    if ctx.attr.debug:
        args.append("--verbose --verbose")
    elif ctx.attr.verbose:
        args.append("--verbose")

    if ctx.attr.format:
        args.append("--format={}".format(ctx.attr.format))

    if ctx.attr.hang_closing:
        args.append("--hang-closing")

    if ctx.attr.ignore:
        args.append("--ignore={}".format(",".join(ctx.attr.ignore)))

    if ctx.attr.max_doc_length > 0:
        args.append("--max-doc-length={}".format(ctx.attr.max_doc_length))

    if ctx.attr.max_line_length > 0:
        args.append("--max-line-length={}".format(ctx.attr.max_line_length))

    if ctx.attr.quiet:
        args.append("--quiet")

    if ctx.attr.select:
        args.append("--select={}".format(",".join(ctx.attr.select)))

    if ctx.attr.show_pep8:
        args.append("--show-pep8")

    if ctx.attr.show_source:
        args.append("--show-source")

    if ctx.attr.statistics:
        args.append("--statistics")

    pycodestyle = shell.quote(
        ctx.attr._pycodestyle[DefaultInfo].files_to_run.executable.short_path,
    )
    srcs = [shell.quote(f.short_path) for f in ctx.files.srcs]

    substitutions = {
        "@@ARGS@@": " ".join(args),
        "@@GENERATED_FILE_MARKER@@": "GENERATED FILE DO NOT EDIT",
        "@@PYCODESTYLE@@": pycodestyle,
        "@@SRCS@@": " ".join(srcs),
    }

    ctx.actions.expand_template(
        template = ctx.file._template,
        output = ctx.outputs.executable,
        substitutions = substitutions,
        is_executable = True,
    )

    runfiles = ctx.runfiles(files = files).merge(
        ctx.attr._pycodestyle[DefaultInfo].default_runfiles,
    )

    return [DefaultInfo(runfiles = runfiles)]

_PY_PYCODESTYLE_TEST_ATTRS = {
    "srcs": attr.label_list(
        doc = "The list of source files that are to be processed",
        allow_empty = False,
        allow_files = [".py"],
    ),
    "config": attr.label(
        allow_single_file = [".cfg", ".ini"],
        doc = "The name of the user config file. This must be a 'setup.cfg' " +
              "or 'tox.ini' file with a `[pycodestyle]` section.",
    ),
    "count": attr.bool(
        doc = "Print the total number of errors and warnings to standard error",
    ),
    "debug": attr.bool(doc = "Print debug messages"),
    "format": attr.string(
        doc = "The error format",
        values = ["", "default", "pylint"],
    ),
    "hang_closing": attr.bool(
        doc = "Hang the closing bracket instead of matching the indentation " +
              "of the line of the opening bracket",
    ),
    "ignore": attr.string_list(doc = "The list of errors and warnings to skip"),
    "max_doc_length": attr.int(doc = "The maximum allowed doc line length"),
    "max_line_length": attr.int(doc = "The maximum allowed line length"),
    "quiet": attr.bool(doc = "Report only file names"),
    "select": attr.string_list(
        doc = "The list of errors and warnings to select",
    ),
    "show_pep8": attr.bool(doc = "Show the text of PEP 8 for each error"),
    "show_source": attr.bool(doc = "Show the source code for each error"),
    "statistics": attr.bool(doc = "Count errors and warnings"),
    "verbose": attr.bool(doc = "Print status messages"),
    "_pycodestyle": attr.label(
        default = "//tools/rt/pycodestyle",
        doc = "The name of the pycodestyle console script",
        cfg = "host",
    ),
    "_template": attr.label(
        default = "//tools/rt/pycodestyle:pycodestyle_runner.bash.in",
        doc = "The name of the pycodestyle test runner template",
        allow_single_file = True,
    ),
}  # buildifier: disable=unsorted-dict-items

_py_pycodestyle_test = rule(
    implementation = _py_pycodestyle_test_impl,
    attrs = _PY_PYCODESTYLE_TEST_ATTRS,
    doc = "Check compliance with Python code style conventions, such as PEP 8",
    provides = [DefaultInfo],
    test = True,
)

def py_pycodestyle_test(**kwargs):
    """Check compliance with Python style conventions, such as PEP 8.

    Args:
        name: A unique name for this test target.
        srcs: The list of source (".py") files that are to be processed.
        config: The name of the user config file. This must be a "setup.cfg" or
            "tox.ini" file with a `[pycodestyle]` section.
        count: Print the total number of errors and warnings to standard error.
        debug: Print debug messages.
        format: The error format (if specified, must be `default` or `pylint`).
        hang_closing: Hang the closing bracket instead of matching the
            indentation of the line of the opening bracket.
        ignore: The list of errors and warnings to skip.
        max_doc_length: The maximum allowed doc line length.
        max_line_length: The maximum allowed line length.
        quiet: Report only file names.
        select: The list of errors and warnings to select.
        show_pep8: Show the text of PEP 8 for each error.
        show_source: Show the source code for each error.
        statistics: Count errors and warnings.
        verbose: Print status messages.
    """
    size = kwargs.get("size", "small")
    kwargs["size"] = size
    tags = kwargs.get("tags", [])
    if "block-network" not in tags:
        tags.append("block-network")
    if "requires-network" in tags:
        tags.remove("requires-network")
    kwargs["tags"] = tags
    _py_pycodestyle_test(**kwargs)
