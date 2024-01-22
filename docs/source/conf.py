# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'mplib'
copyright = '2024, minghua, jiayuan, kolin, xinsong'
author = 'minghua, jiayuan, kolin, xinsong'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
  'sphinx.ext.autodoc',
  'sphinx.ext.autosummary',
  'sphinx.ext.duration',
  'sphinx.ext.napoleon',
  "myst_parser",
]

templates_path = ['_templates']
exclude_patterns = []

import os
import sys
import shutil
sys.path.insert(0, os.path.abspath(os.path.join('..', '..', 'mplib')))

def copy_readme_gif(app, docname):
  if app.builder.name == 'html':
    print(app.srcdir)
    source = os.path.join(app.srcdir, '..', '..', 'demo.gif')
    target = os.path.join(app.outdir, 'demo.gif')
    shutil.copyfile(source, target)
  
def setup(app):
  app.connect('build-finished', copy_readme_gif)

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']
