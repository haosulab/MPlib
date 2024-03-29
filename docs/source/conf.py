# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "mplib"
author = "Minghua Liu, Jiayuan Gu, Kolin Guo, Xinsong Lin"
copyright = f"2021-2024, {author}. All rights reserved."
release = "0.1.0"

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
