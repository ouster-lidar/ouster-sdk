import sphinx_rtd_theme # noqa

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'Ouster Python SDK'
copyright = '2021, Ouster, Inc.'
author = 'Ouster SW'

# The full version, including alpha/beta/rc tags
release = '0.2.2-dev0'
version = '0.2.2-dev0'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'sphinx_autodoc_typehints',
    'sphinx_rtd_theme',
    'sphinx_copybutton',
    'sphinx_tabs.tabs',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'logo_only': True,
    'display_version': True,
    # 'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    # 'vcs_pageview_mode': '',
    # 'style_nav_header_background': 'rgb(29, 29, 29)',
    # Toc options
    # 'collapse_navigation': True,
    # 'sticky_navigation': True,
    'navigation_depth': 2,
    'includehidden': True,
    # 'titles_only': False
}

html_context = {
    'display_github': True,
    'github_user': 'ouster-lidar',
    'github_repo': 'ouster_example',
    # 'github_version': 'ouster/python-bindings',
    'github_version': 'master',
    'conf_py_path': '/python/docs/'
}

# show Ouster logo in sidebar header
html_logo = 'images/Ouster_Logo_TM_Stacked_White_RGB.svg'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Ouster tweaks to the theme to handle logo size, fonts, etc.
html_css_files = [
    'css/ouster_rtd_tweaks.css',
]


# -- Extension configuration -------------------------------------------------

# use both class and constructor docstrings
autoclass_content = 'both'

# do not alphabetize class members
autodoc_member_order = 'bysource'

# tell autodoc not to output fully-qualified names
add_module_names = False

# don't parse numpy-style docstrings
napoleon_numpy_docstring = False

# does not seem to work
# autodoc_type_aliases = {
#     'BufferT': 'BufferT',
#     'Packet': 'Packet',
# }

# napoleon_use_param = False

# ----- Todos Configs ------
todo_include_todos = True
todo_link_only = True
todo_emit_warnings = True

# copybutton configs
# Note: last entry treats four spaces as a prompt to support "continuation lines"
copybutton_prompt_text = r'>>> |\.\.\. |\$ |PS > |C:\\> |> |    '
copybutton_prompt_is_regexp = True

# tabs behavior
sphinx_tabs_disable_tab_closing = True
