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
        sha256 = "ec064a5edd2a2a210cf8162305869a27b3ed6c7e50caa70687bc9d72177f61f3",
        urls = ["https://github.com/bazelbuild/buildtools/releases/download/1.0.0/buildifier"],
    )

    http_file(
        name = "buildifier_macos",
        downloaded_file_path = "buildifier",
        executable = True,
        sha256 = "6e7545fdfd4b142041b4fefd8f338b31dee556aa37e9fa8e244ee66a765d352f",
        urls = ["https://github.com/bazelbuild/buildtools/releases/download/1.0.0/buildifier.mac"],
    )
