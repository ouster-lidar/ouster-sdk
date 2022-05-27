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
import subprocess
import tempfile
from string import Template
import shutil
import os
import sys

project = 'Ouster Python SDK'
copyright = '2022, Ouster, Inc.'
author = 'Ouster SW'

# The full version, including alpha/beta/rc tags
version = '0.4.0'
release = '0.4.0'

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
    'breathe'
]

# Full path generated Doxygen XML dir resolved in do_doxygen_generate_xml()
# handler below
breathe_projects = {'cpp_api': "xml"}

breathe_default_project = 'cpp_api'
breathe_show_include = True
breathe_default_members = ()

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
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    # 'titles_only': False
}

html_context = {
    'display_github': True,
    'github_user': 'ouster-lidar',
    'github_repo': 'ouster_example',
    # 'github_version': 'ouster/python-bindings',
    'github_version': 'master',
    'conf_py_path': '/docs/'
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
todo_include_todos = False
todo_link_only = True
todo_emit_warnings = False

# copybutton configs
# Note: last entry treats four spaces as a prompt to support "continuation lines"
copybutton_prompt_text = r'>>> |\.\.\. |\$ |PS > |C:\\> |> |    '
copybutton_prompt_is_regexp = True

# tabs behavior
sphinx_tabs_disable_tab_closing = True


# -- Doxygen XML generation handlers -----------------------------------

def do_doxygen_generate_xml(app):

    # Only runs is breathe projects exists
    if not app.config["breathe_projects"]:
        return

    if shutil.which("doxygen") is None:
        raise SystemError(
            "Expects 'doxygen' command on the PATH to generate C++ docs")

    print("===== Generating Doxygen XML ======")

    # Prepare temp Doxyfile with resolved param values
    temp_doxy_file_dir = tempfile.mkdtemp()
    temp_doxy_file = os.path.join(temp_doxy_file_dir, "Doxyfile")
    doxygen_output_dir = app.doctreedir
    dictionary = {
        'project': app.config.project,
        'version': app.config.release,
        'output_dir': doxygen_output_dir
    }

    with open(os.path.join(app.confdir, 'Doxyfile'), 'r') as template_file:
        template = Template(template_file.read())
        with open(temp_doxy_file, 'w') as template_file_out:
            template_file_out.write(template.substitute(dictionary))

    # Store for later cleanup
    app.config.add("temp_doxy_file_dir", temp_doxy_file_dir, "env", [])

    subprocess.call(["doxygen", temp_doxy_file], cwd=app.confdir)

    # Update breathe_projects paths to be relative from actual doxygen_output_dir
    for name, path in app.config["breathe_projects"].items():
        app.config["breathe_projects"].update(
            {name: os.path.join(doxygen_output_dir, path)})

def do_doxygen_temp_cleanup(app, exception):
    if "temp_doxy_file_dir" in app.config:
        shutil.rmtree(app.config["temp_doxy_file_dir"])

def setup(app):

    # Add a hook for generating doxygen xml and cleaning up
    app.connect("builder-inited", do_doxygen_generate_xml)
    app.connect("build-finished", do_doxygen_temp_cleanup)

