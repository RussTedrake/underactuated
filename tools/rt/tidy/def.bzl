# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@bazel_skylib//lib:shell.bzl", "shell")

def _html_tidy_test_impl(ctx):
    args = []
    files = ctx.files.srcs + [ctx.executable._tidy]

    if ctx.file.config != None:
        args.extend(["-config", shell.quote(ctx.file.config.short_path)])
        files = files + [ctx.file.config]

    args.extend(["-output", "/dev/null"])

    tidy = shell.quote(ctx.executable._tidy.short_path)
    srcs = [shell.quote(f.short_path) for f in ctx.files.srcs]

    substitutions = {
        "@@ARGS@@": " ".join(args),
        "@@GENERATED_FILE_MARKER@@": "GENERATED FILE DO NOT EDIT",
        "@@SRCS@@": " ".join(srcs),
        "@@TIDY@@": tidy,
    }

    ctx.actions.expand_template(
        template = ctx.file._template,
        output = ctx.outputs.executable,
        substitutions = substitutions,
        is_executable = True,
    )

    return [DefaultInfo(runfiles = ctx.runfiles(files = files))]

_HTML_TIDY_TEST_ATTRS = {
    "srcs": attr.label_list(
        doc = "The list of source files that are to be processed",
        allow_empty = False,
        allow_files = [".html", ".xml"],
    ),
    "config": attr.label(
        allow_single_file = True,
        doc = "Set configuration options from the specified file",
    ),
    "_template": attr.label(
        default = "//tools/rt/tidy:tidy_runner.bash.in",
        doc = "The name of the tidy test runner template",
        allow_single_file = True,
    ),
    "_tidy": attr.label(
        default = "//tools/rt/tidy",
        doc = "The name of the tidy executable",
        executable = True,
        cfg = "host",
    ),
}  # buildifier: disable=unsorted-dict-items

_html_tidy_test = rule(
    implementation = _html_tidy_test_impl,
    attrs = _HTML_TIDY_TEST_ATTRS,
    doc = "Validate, correct, and pretty-print HTML/XHTML/XML",
    provides = [DefaultInfo],
    test = True,
)

def html_tidy_test(**kwargs):
    """Utility to clean up and pretty print HTML/XHTML/XML.

    Args:
        name: A unique name for this test target.
        srcs: The list of source (".html", ".xml") files that are to be
            processed.
        config: Set configuration options from the specified file.
    """

    size = kwargs.get("size", "small")
    kwargs["size"] = size
    tags = kwargs.get("tags", [])
    if "block-network" not in tags:
        tags.append("block-network")

    # TODO(jamiesnape): Work out why tidy fails when sandboxed.
    if "no-sandbox" not in tags:
        tags.append("no-sandbox")
    if "requires-network" in tags:
        tags.remove("requires-network")
    kwargs["tags"] = tags
    _html_tidy_test(**kwargs)
