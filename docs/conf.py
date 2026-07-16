"""
Sphinx configuration for Ouster SDK documentation builder.

This file only contains a selection of the most common options. For a full
list see the documentation:
https://www.sphinx-doc.org/en/master/usage/configuration.html
"""
from __future__ import annotations

import os
import posixpath
import re
import sys
from pathlib import Path
from typing import Any, Dict, List

import sphinx_press_theme
from sphinx import addnodes
from sphinx.util.osutil import relative_uri

# -----------------------------------------------------------------------------
# Patch Sphinx to apply PR #13836 fix for nanobind methods
# -----------------------------------------------------------------------------
# PR #13836: Ensure nanobind nb_method is treated like instancemethod
# (not as attribute descriptor), so autodoc renders method signatures correctly.
# https://github.com/sphinx-doc/sphinx/pull/13836
import sphinx.util.inspect as sui

_orig_isattributedescriptor = sui.isattributedescriptor


def _isattributedescriptor_patched(obj):
    """Monkey-patched version that treats nanobind nb_method like instancemethod."""
    try:
        unwrapped = sui.unwrap_all(obj)
        if type(unwrapped).__name__ == "nb_method":
            return False
    except Exception:
        pass
    return _orig_isattributedescriptor(obj)


sui.isattributedescriptor = _isattributedescriptor_patched

# -----------------------------------------------------------------------------
# Path setup: add _ext which has docs helpers
# -----------------------------------------------------------------------------
DOCS_DIR = Path(__file__).parent.resolve()
EXT_DIR = DOCS_DIR / "_ext"
if str(EXT_DIR) not in sys.path:
    sys.path.insert(0, str(EXT_DIR))

SNIPPETS_DIR = DOCS_DIR / "_snippets"
if SNIPPETS_DIR.exists() and str(SNIPPETS_DIR) not in sys.path:
    sys.path.insert(0, str(SNIPPETS_DIR))
# Allow doctests to import python snippets that live in nested directories
for extra_snippet_dir in SNIPPETS_DIR.glob("**/python"):
    if extra_snippet_dir.is_dir():
        path_str = str(extra_snippet_dir)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)


# -----------------------------------------------------------------------------
# Environment context
# -----------------------------------------------------------------------------
base_url = os.environ.get("docs_url", "")
# sphinx-sitemap reads html_baseurl;
html_baseurl = base_url.rstrip("/") if base_url else ""
link_versions_raw = os.getenv("LINK_VERSIONS", "latest")
if link_versions_raw is None:
    raise RuntimeError(f"Link versions not defined {link_versions_raw}")
link_versions: List[str] = link_versions_raw.split(",")

enhanced_warnings = os.getenv("DOXYGEN_ENHANCED_WARNINGS", "false").lower() == "true"

# use SDK source location from environment or try to guess
SRC_PATH = os.path.dirname(os.path.abspath(__file__))
OUSTER_SDK_PATH = os.getenv('OUSTER_SDK_PATH')
if OUSTER_SDK_PATH is None:
    OUSTER_SDK_PATH = os.path.join(SRC_PATH, "sdk")
if not os.path.exists(OUSTER_SDK_PATH):
    OUSTER_SDK_PATH = os.path.dirname(SRC_PATH)
if not os.path.exists(os.path.join(OUSTER_SDK_PATH, "cmake")):
    raise RuntimeError("Could not guess OUSTER_SDK_PATH")

# -----------------------------------------------------------------------------
# Project metadata
# -----------------------------------------------------------------------------
project = "Ouster Sensor SDK"
author = "Ouster SW"
copyright = "2026, Ouster"


def _read_version() -> str:
    repo_root = DOCS_DIR.parent
    for filename in ("VERSION.generated", "VERSION"):
        version_path = repo_root / filename
        if not version_path.is_file():
            continue
        ns: dict = {}
        try:
            exec(version_path.read_text(encoding="utf-8"), ns)
        except Exception as e:
            raise RuntimeError(f"Unable to read {version_path}: {e}") from e
        return ns["__version__"]
    raise RuntimeError(f"Neither VERSION.generated nor VERSION found under {repo_root}")


version = release = _read_version()

IS_EXTERNAL_DOCS = os.getenv("IS_EXTERNAL", "").lower() == "true"
INTERNAL_INSTALL_COMMAND = (
    "python3 -m pip install --extra-index-url=https://packages.sfo.ouster.io/sw/pypi/simple "
    "--pre --force ouster-sdk"
)

tags: set[str] = set()
if IS_EXTERNAL_DOCS:
    install_unix_upgrade = "python3 -m pip install --upgrade ouster-sdk"
    install_unix_fresh = "python3 -m pip install ouster-sdk"
    install_windows_upgrade = "py -3 -m pip install --upgrade ouster-sdk"
    install_windows_fresh = "py -3 -m pip install ouster-sdk"
else:
    install_unix_upgrade = INTERNAL_INSTALL_COMMAND
    install_unix_fresh = INTERNAL_INSTALL_COMMAND
    install_windows_upgrade = INTERNAL_INSTALL_COMMAND.replace("python3 -m", "py -3 -m", 1)
    install_windows_fresh = install_windows_upgrade
    # Sphinx injects tags into conf.py; ignore mypy error, tags are preserved.
    tags.add("internal_docs")  # type: ignore[name-defined]  # noqa: F821

rst_epilog = "\n".join(
    [
        f".. |install_unix_upgrade| replace:: {install_unix_upgrade}",
        f".. |install_unix_fresh| replace:: {install_unix_fresh}",
        f".. |install_windows_upgrade| replace:: {install_windows_upgrade}",
        f".. |install_windows_fresh| replace:: {install_windows_fresh}",
        "",
    ]
)

# -----------------------------------------------------------------------------
# Extensions
# -----------------------------------------------------------------------------
ouster_no_cpp_api = os.environ.get("OUSTER_DOCS_NO_CPP_API", "").strip() not in ("", "0")
ouster_no_python_api = os.environ.get("OUSTER_DOCS_NO_PYTHON_API", "").strip() not in ("", "0")

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.todo",
    "sphinx.ext.doctest",
    "sphinx_autodoc_typehints",
    "sphinx_copybutton",
    "sphinx_tabs.tabs",
    "sphinx_design",
    "sphinx_new_tab_link",
    "sphinx.ext.graphviz",
]

if html_baseurl:
    extensions.append("sphinx_sitemap")

extensions += [
    "auto_api_generator",
    "tabbed_api_links",
    "ouster_post_build",
]

if not ouster_no_cpp_api:
    extensions += ["breathe", "exhale", "exhale_group_function_fix"]

# -----------------------------------------------------------------------------
# General configuration
# -----------------------------------------------------------------------------
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store',
                    'cli']
if ouster_no_cpp_api:
    exclude_patterns += ['cpp']
if ouster_no_python_api:
    exclude_patterns += ['python']


# Autodoc / Napoleon
# use both class and constructor docstrings
autoclass_content = "both"
# do not alphabetize class members
autodoc_member_order = "bysource"
autodoc_typehints = "both"
# tell autodoc not to output fully-qualified names
add_module_names = False
# don't parse numpy-style docstrings
napoleon_numpy_docstring = False
numpydoc_show_class_members = False

# TODOs
todo_include_todos = False
todo_emit_warnings = True
todo_link_only = True

# copybutton configs
copybutton_selector = "div.highlight pre, pre.literal-block"
copybutton_prompt_text = r"PS >|^\$ "
copybutton_prompt_is_regexp = True
copybutton_exclude = '.linenos, .gp'

# tabs behavior
sphinx_tabs_disable_tab_closing = True

# external link config
new_tab_link_show_external_link_icon = True

# -----------------------------------------------------------------------------
# Breathe / Exhale (C++ API)
# -----------------------------------------------------------------------------
breathe_projects = {'cpp_api': "xml"}
breathe_default_project = 'cpp_api'
breathe_show_include = True
breathe_default_members = ()
breathe_show_define_initializer = True
breathe_show_enumvalue_initializer = True

breathe_projects_source = {
    "cpp_api": ("../", [
        "ouster_algorithm",
        "ouster_core",
        "ouster_library",
        "ouster_mapping",
        "ouster_osf",
        "ouster_pcap",
        "ouster_perception",
        "ouster_sensor",
        "ouster_viz"
    ])
}
cpp_id_attributes = ['id']
cpp_paren_attributes = ['paren']

exhale_output_folder = "cpp/api_cpp"
exhale_cpp_dir = exhale_output_folder.split("/")[0]
exhale_api_dir = exhale_output_folder.split("/")[1]

exhale_args = {
    "containmentFolder": f"./{exhale_output_folder}",
    "rootFileName": "namespace_ouster.rst",
    "rootFileTitle": "C++ API Reference",
    "pageHierarchySubSectionTitle": "",
    "fullApiSubSectionTitle": "",
    "doxygenStripFromPath": ".",
    "createTreeView": True,
    "afterTitleDescription": """
    This section provides a comprehensive reference for the Ouster C++ SDK.
    Navigate through the hierarchical structure below to explore classes, namespaces, and files.
    """,
    "fullToctreeMaxDepth": 4,
    "listingExclude": [
        r'.*namespace_ChanField.*',
        r'.*ChanField.*',
    ],
    "unabridgedOrphanKinds": [
        "class", "function", "define", "typedef", "enum", "enumvalue",
        "file", "variable", "union", "dir", "page", "group"
    ],
    # Disable auto-generated local tables of contents from Exhale output pages.
    "kindsWithContentsDirectives": ["namespace", "class", "struct"],
    "contentsDirectives": False,
    # Use only for debugging:
    # "verboseBuild": False,
    # https://exhale.readthedocs.io/en/latest/reference/configs.html#exhale.configs.generateBreatheFileDirectives
    # "generateBreatheFileDirectives": True,
}

# ------------------------e-----------------------------------------------------
# HTML theme & assets
# -----------------------------------------------------------------------------
html_logo = f'{base_url}/_static/Ouster_Logo_TM_Horiz_White_RGB.svg'
html_static_path = ['_static']
html_favicon = f'{base_url}/_static/oust_icon_only.ico'

_BASE_HTML_CSS_FILES = [
    'css/right-rail.css',
    'css/cpp_autosummary.css',
    'css/python_autosummary.css',
    'css/press_overrides.css',
    'css/python_param_fields.css',
    'css/page_loading.css'
]

DOCS_DIR = Path(__file__).parent.resolve()
html_theme = 'ouster_press'
html_theme_path = [str(DOCS_DIR / "_themes")]
html_css_files = list(_BASE_HTML_CSS_FILES)
html_theme_options: Dict[str, Any] = {}
html_sidebars: Dict[str, Any] = {}


print(f"Using HTML theme: {html_theme}")
print(f"HTML theme options: {html_theme_options}")

# Set IS_INTERNAL to true for Gitlab redirects on snippets
IS_INTERNAL_DOCS = os.getenv("IS_INTERNAL", "").lower() == "true"
IS_EXTERNAL_DOCS = os.getenv("IS_EXTERNAL", "").lower() == "true"

# Internal/MR/local doc builds (anything except explicit public release).
if not IS_EXTERNAL_DOCS:
    rst_epilog += (
        ".. |internal_installation_link| replace:: For latest version details, "
        "see the :ref:`Ouster SDK installation guide <ouster-sdk-installation-commands>`.\n"
    )

SNIPPET_ENV_JS = DOCS_DIR / "_static" / "js" / "snippet_env_config.js"
SNIPPET_ENV_JS.write_text(
    "window.DOCS_IS_INTERNAL = " + ("true" if IS_INTERNAL_DOCS else "false") + ";\n",
    encoding="utf-8"
)

html_js_files = ['js/page_ready.js',
                 'js/cpp_autosummary.js',
                 'js/cpp_namespace_toctree.js',
                 'js/filter_doxygen_tags.js',
                 'js/snippet_env_config.js',
                 'js/snippet_github_button.js',
                 'js/copy_heading_link.js',
                 'js/sidebar-toggle.js',
                 'js/right_toc_highlight.js',
                 'js/right_toc_toggle.js',
                 'js/searchbox_placeholder.js',
                 'js/tabbed-links.js']

MIGRATION_GUIDE_LINKS = [
    ("Migrating from 0.16.2 to 1.0.0", "migration/migration-0.16.2-1.0"),
    ("Migrating from 0.15.0 to 0.16.0", "migration/migration-0.15.0-0.16.0"),
    ("Migrating from 0.14.0 to 0.15.0", "migration/migration-0.14.0-0.15.0"),
    ("Migration from 0.13.1 to 0.14.0", "migration/migration-20241004-20250113"),
    ("Migration from 0.10.0 to 0.11.0", "migration/migration-20231031-20240423"),
    ("Migrating from 0.7.1 to 0.8.1", "migration/migration-20230114-20230403"),
    ("Migrating from 0.5.1 to 0.7.1", "migration/migration-20220927-20230114"),
]


def _load_migration_guides():
    """Collect migration guide labels and docnames for navbar dropdown."""
    guides = []
    for title, docname in MIGRATION_GUIDE_LINKS:
        guides.append({
            "title": title,
            "docname": docname,
        })
    return guides


def _version_urls(base):
    base_clean = base.rstrip("/")
    if base_clean.endswith(".html"):
        latest_url = base_clean
        base_dir = posixpath.dirname(base_clean)
    else:
        base_dir = base_clean
        latest_url = f"{base_dir}/index.html" if base_dir else "index.html"
    return base_dir, latest_url


html_context: Dict[str, Any] = {
    "display_github": True,
    "github_user": "ouster-lidar",
    "github_repo": "ouster-sdk",
    "github_version": "master",
    "conf_py_path": "/docs/",
    "versions": [],
    "current_version": "latest",
}
base_dir, latest_link = _version_urls(base_url)
html_context["versions"].append(["latest", latest_link])
for v in link_versions:
    version_link = f"{base_dir}/{v}/index.html" if base_dir else f"{v}/index.html"
    html_context["versions"].append([v, version_link])

html_context["migration_guides"] = _load_migration_guides()
html_context["navbar_links"] = [
    {
        "label": "Documentation",
        "docname": "index",
    },
    {
        "label": "Sensor Docs",
        "url": "https://static.ouster.dev/sensor-docs/index.html",
    },
    {
        "label": "API Reference",
        "docname": "python/api_python/ouster.sdk",
        "new_tab": True,
    },
    {
        "label": "Community Forum",
        "url": "https://community.ouster.com/tag/sdk",
    },
]
html_context["navbar_github_url"] = "https://github.com/ouster-lidar/ouster-sdk"


# -----------------------------------------------------------------------------
# Doctest imports
# -----------------------------------------------------------------------------
try:
    import doctest as _doctest
    doctest_default_flags = _doctest.ELLIPSIS | _doctest.NORMALIZE_WHITESPACE
except Exception:
    pass


# -----------------------------------------------------------------------------
# Post processing for exhale output
# -----------------------------------------------------------------------------
def _collect_sections(app, docname: str, base_uri: str) -> list[dict]:
    toc_info = app.env.toc_dict.get(docname, {})
    sections = []
    doc_title_node = app.env.titles.get(docname)
    doc_title = doc_title_node.astext() if doc_title_node else ''
    for section in toc_info.get('sections', []):
        title_text = section['title']
        # Skip section if it duplicates the document title (prevents double first entry)
        if doc_title and title_text.strip().lower() == doc_title.strip().lower():
            continue
        href = section['href']
        anchor = href.lstrip('#') if href else ''
        sections.append({
            'name': docname,
            'title': title_text,
            'current': False,
            'children': [],
            'ext_resource': False,
            'docname': docname,
            'anchor': anchor,
            'href': f"{base_uri}{href}",
        })
    return sections


def _collect_child_docs(app, parent_docname: str, current_docname: str) -> list[dict]:
    """Return nested documents linked from ``parent_docname`` toctrees."""

    nested_docs: list[dict] = []
    toc_info = app.env.toc_dict.get(parent_docname, {})
    parent_base = parent_docname.rsplit('/', 1)[0] if '/' in parent_docname else ''

    for nested in toc_info.get('toctrees', []):
        for nested_title, target in nested['entries']:
            if not target or '://' in target:
                continue

            anchor = ''
            doc_target = target
            if '#' in target:
                doc_target, anchor = target.split('#', 1)

            if doc_target.startswith('/'):
                doc_target = doc_target.lstrip('/')
            elif '/' not in doc_target and parent_base:
                doc_target = posixpath.normpath(posixpath.join(parent_base, doc_target))

            title_node = app.env.titles.get(doc_target) or app.env.titles.get(parent_docname)
            display_title = nested_title or (title_node.astext() if title_node else doc_target)

            nested_docs.append({
                'docname': doc_target,
                'anchor': anchor,
                'title': display_title,
                'current': (doc_target == current_docname),
            })

    return nested_docs


def patched_add_toctree_data(app, pagename, templatename, context, doctree, *, maxdepth=4):
    master = app.env.get_doctree(app.env.config.master_doc)
    res = []
    for tree in master.traverse(addnodes.toctree):
        is_hidden = bool(tree.get('hidden'))
        entries = list(tree.get('entries', []))
        if not entries:
            continue

        if len(entries) == 1:
            entry = entries[0]
            if len(entry) < 2:
                continue

            entry_title, entry_docname = entry[0], entry[1]
            # Keep API root pages as a single parent entry; children come from
            # _collect_child_docs (Python ouster.sdk, C++ namespace roots).
            if entry_docname not in (
                'python/api_python/ouster.sdk',
                f'{exhale_output_folder}/namespace_ouster',
            ):
                toc_entry = app.env.toc_dict.get(entry_docname)
                if not toc_entry:
                    continue

                nested = list(toc_entry.get('toctrees', []))
                if nested:
                    original_caption = tree.get('caption') or entry_title
                    tree = nested[0]
                    if original_caption and not tree.get('caption'):
                        tree['caption'] = original_caption
                    entries = list(tree.get('entries', []))
                    if not entries:
                        continue

        if is_hidden:

            def _is_api_entry(entry_tuple):
                if len(entry_tuple) < 2:
                    return False
                target = entry_tuple[1]
                return (
                    target.startswith('python/api_python/') or
                    target.startswith(f"{exhale_output_folder}/")
                )

            if not any(_is_api_entry(entry) for entry in entries):
                continue

        current0 = False
        rendered_entries = []
        for entry in entries:
            if len(entry) < 2:
                continue

            title, name = entry
            if not title:
                title = app.env.titles[name].astext()
            ext_resource = '://' in name
            current1 = (pagename == name)
            if current1:
                current0 = True

            children: list[dict] = []
            has_l2_children = False
            if not ext_resource:
                child_docs = _collect_child_docs(app, name, pagename)
                if child_docs:
                    children.extend(child_docs)
                    has_l2_children = True

                child_is_current = any(child['current'] for child in child_docs)
                if child_is_current and not current1:
                    current1 = True
                    current0 = True

                # Only add section anchors when we're rendering the same page
                if pagename == name:
                    children.extend(_collect_sections(app, name, ''))

            entry_payload = {
                'name': name,
                'title': title,
                'current': current1,
                'children': children,
                'ext_resource': ext_resource,
                'has_l2_children': has_l2_children,
            }
            if not ext_resource:
                entry_payload['docname'] = name

            rendered_entries.append(entry_payload)

        toc_docname = tree['parent']
        toc_section = tree.parent.parent
        anchor_id = ''
        if hasattr(toc_section, 'ids') and toc_section['ids']:
            anchor_id = toc_section['ids'][0]
        section_name = ''
        if hasattr(toc_section, 'names') and toc_section['names']:
            section_name = toc_section['names'][0]
        title = tree['caption'] or section_name

        base = app.builder.get_target_uri(pagename).rsplit('#', 1)[0]
        target = app.builder.get_target_uri(toc_docname).rsplit('#', 1)[0]
        href = relative_uri(base, target)
        if anchor_id:
            href = f"{href}#{anchor_id}"

        res.append({
            'docname': toc_docname,
            'href': href,
            'title': title,
            'current': current0,
            'entries': rendered_entries,
            'hidden': is_hidden,
        })

    context['toctree_data'] = res

    if (
        pagename.startswith('python/api_python/') or
        pagename.startswith(f"{exhale_output_folder}/") or
        pagename.startswith('_modules/')
    ):
        toc_html = context.get('toc')
        if toc_html:
            patterns = [
                '<li>\\s*<a[^>]*href="#module-[^"]+"[^>]*>Module contents</a>\\s*</li>',
                '<li>\\s*<a[^>]*href="#subpackages"[^>]*>Subpackages</a>\\s*</li>',
                '<li>\\s*<a[^>]*href="#submodules"[^>]*>Submodules</a>\\s*</li>',
            ]
            for pattern in patterns:
                toc_html = re.sub(pattern, '', toc_html, flags=re.IGNORECASE)
            context['toc'] = toc_html

    _add_page_nav_sections(app, pagename, templatename, context, doctree)


sphinx_press_theme.add_toctree_data = patched_add_toctree_data


# --------------------------------------------------------------------------------
# Page-nav section names: caption of owning sidebar group or use L1 sidebar title
# if group has L2 children.
# Inject into context as prev_section/next_section for use in page-nav.html.
# --------------------------------------------------------------------------------


def _link_from_rel(rel):
    """Get link from prev/next (can be dict or object)."""
    if not rel:
        return None
    return rel.get("link") if isinstance(rel, dict) else getattr(rel, "link", None)


def _link_to_docname(link, pagename):
    """Resolve relative prev/next link to absolute docname (no .html, no #)."""
    if not link:
        return None
    clean = link.replace("\\", "/").split("#", 1)[0].removesuffix(".html").strip("/")
    if not clean:
        return None
    base = pagename.rsplit("/", 1)[0] if "/" in pagename else ""
    combined = f"{base}/{clean}" if base else clean
    normalized = posixpath.normpath(combined)
    return normalized if normalized != "." else None


def _collect_descendant_docnames(entry):
    """Yield all docnames for an entry and its descendants (L2, L3, ...)."""
    docname = entry.get("docname") or entry.get("name")
    if docname:
        yield docname
    for child in entry.get("children") or []:
        yield from _collect_descendant_docnames(child)


def _build_docname_to_l1_title(toctree_data):
    """Build map: docname -> section label from sidebar groups.

    If a sidebar-group has L2 entries, use the owning L1 title.
    If a sidebar-group has no L2 entries, use the group caption/title.
    """
    doc_to_l1 = {}
    for toc in toctree_data or []:
        entries = toc.get("entries") or []
        if not entries:
            continue
        group_title = toc.get("title")
        group_has_l2 = any(entry.get("has_l2_children") for entry in entries)
        for entry in entries:
            section_title = entry.get("title") if group_has_l2 else group_title
            if not section_title:
                continue
            for doc in _collect_descendant_docnames(entry):
                doc_to_l1[doc] = section_title
    return doc_to_l1


def _docname_to_section(docname, doc_to_l1):
    """Resolve docname to section title by exact, then path-prefix matches."""
    if not docname or not doc_to_l1:
        return None
    parts = docname.split("/")
    candidates = ["/".join(parts[:i]) for i in range(len(parts), 0, -1)]
    for candidate in candidates:
        section = doc_to_l1.get(candidate)
        if section:
            return section
    return None


def _resolve_nav_target(rel, pagename, doc_to_l1):
    """Resolve prev/next relation to (link, docname, section)."""
    link = _link_from_rel(rel)
    docname = _link_to_docname(link, pagename) if link else None
    section = _docname_to_section(docname, doc_to_l1) if docname else None
    return link, docname, section


def _add_page_nav_sections(app, pagename, templatename, context, doctree):
    """Inject prev_section/next_section from sidebar-group + L1/L2 structure."""
    prev = context.get("prev")
    next_ = context.get("next")
    toctree_data = context.get("toctree_data")
    if not toctree_data:
        return

    doc_to_l1 = _build_docname_to_l1_title(toctree_data)

    prev_link, prev_docname, prev_section = _resolve_nav_target(prev, pagename, doc_to_l1)
    next_link, next_docname, next_section = _resolve_nav_target(next_, pagename, doc_to_l1)

    context.update({
        "prev_section": prev_section,
        "next_section": next_section,
    })


def setup(app):
    return {"version": "0.1", "parallel_read_safe": True}
