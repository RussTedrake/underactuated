# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

project = "underactuated"
copyright = "2023, Russ Tedrake"
author = "Russ Tedrake"

templates_path = ["_templates"]
exclude_patterns = ["**/*.md"]

extensions = [
    "myst_parser",  # To enable autodoc from markdown
    "sphinx.ext.autodoc",
    "sphinx.ext.mathjax",
    "sphinx.ext.napoleon",
    "sphinx.ext.intersphinx",
]

napoleon_include_init_with_doc = True

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_css_files = [
    "custom.css",
]

autodoc_typehints = "description"

intersphinx_mapping = {
    "drake": ("https://drake.mit.edu/pydrake", None),
}
