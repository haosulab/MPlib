# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import re

project = "mplib"
author = "Minghua Liu, Jiayuan Gu, Kolin Guo, Xinsong Lin"
copyright = f"2021-2024, {author}. All rights reserved."
git_describe_ret = os.popen("git describe --abbrev=8 --tags --match v*").read().strip()
if "-" in git_describe_ret:  # commit after a tag
    release = "+git.".join(
        re.findall("^v(.*)-[0-9]+-g(.*)", git_describe_ret)[0]
    )  # tag-commithash
else:
    release = git_describe_ret[1:]
version = release

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.duration",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "myst_parser",
    "sphinx_copybutton",
    "sphinxext.opengraph",
]

templates_path = ["_templates"]
exclude_patterns = []

maximum_signature_line_length = 88  # limit maximum method/function signature length
autodoc_preserve_defaults = True
autodoc_default_options = {
    "members": True,
    "member-order": "bysource",
    "inherited-members": True,
    "show-inheritance": True,
    "undoc-members": True,
    "special-members": "__init__",
}


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# https://pradyunsg.me/furo/customisation/
html_theme = "furo"
# html_theme = "sphinx_book_theme"
html_static_path = []
html_theme_options = {
    # "announcement": "<em>Important</em> announcement!",
    # Comment out for Read the Docs
    # "top_of_page_button": "edit",
    # "source_repository": "https://github.com/haosulab/MPlib",
    # "source_branch": "main",
    # "source_directory": "docs/source/",
}
