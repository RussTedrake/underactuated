# -*- mode: python -*-
# vi: set ft=python :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_file")

def buildifier_dependencies():
    http_file(
        name = "buildifier_linux",
        downloaded_file_path = "buildifier",
        executable = True,
        sha256 = "4c985c883eafdde9c0e8cf3c8595b8bfdf32e77571c369bf8ddae83b042028d6",
        urls = ["https://github.com/bazelbuild/buildtools/releases/download/0.29.0/buildifier"],
    )

    http_file(
        name = "buildifier_macos",
        downloaded_file_path = "buildifier",
        executable = True,
        sha256 = "9b108decaa9a624fbac65285e529994088c5d15fecc1a30866afc03a48619245",
        urls = ["https://github.com/bazelbuild/buildtools/releases/download/0.29.0/buildifier.mac"],
    )
