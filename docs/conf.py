"""
Sphinx configuration for Ouster SDK documentation builder.

This file only contains a selection of the most common options. For a full
list see the documentation:
https://www.sphinx-doc.org/en/master/usage/configuration.html
"""
import os
import sys
from pathlib import Path
from bs4 import BeautifulSoup, Comment

# -----------------------------------------------------------------------------
# Path setup: add _ext which has docs helpers
# -----------------------------------------------------------------------------
DOCS_DIR = Path(__file__).parent.resolve()
EXT_DIR = DOCS_DIR / "_ext"
if str(EXT_DIR) not in sys.path:
    sys.path.insert(0, str(EXT_DIR))


# -----------------------------------------------------------------------------
# Environment context
# -----------------------------------------------------------------------------
base_url = os.environ.get("docs_url", "")
link_versions = os.getenv("LINK_VERSIONS", "latest")
if link_versions is None:
    raise RuntimeError(f"Link versions not defined {link_versions}")
else:
    link_versions = link_versions.split(",")

enhanced_warnings = os.getenv("DOXYGEN_ENHANCED_WARNINGS", "false").lower() == "true"
ignore_missing_doc_warnings = os.getenv("CI_MODE", "false").lower() == "true"

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
copyright = "2022, Ouster"


def _read_version() -> str:
    version_path = DOCS_DIR.parent / "VERSION"
    ns: dict = {}
    try:
        exec(version_path.read_text(encoding="utf-8"), ns)
    except Exception as e:
        raise RuntimeError(f"Unable to read VERSION file: {e}")
    return ns["__version__"]


version = release = _read_version()

# -----------------------------------------------------------------------------
# Extensions
# -----------------------------------------------------------------------------
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.todo",
    "sphinx_autodoc_typehints",
    "sphinx_rtd_theme",
    "sphinx_copybutton",
    "sphinx_tabs.tabs",
    "breathe",
    "exhale",
    "sphinx_rtd_size",
    "sphinx.ext.graphviz",
]
extensions += [
    "auto_api_generator",
]


# -----------------------------------------------------------------------------
# General configuration
# -----------------------------------------------------------------------------
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# Autodoc / Napoleon
# use both class and constructor docstrings
autoclass_content = "both"
# do not alphabetize class members
autodoc_member_order = "bysource"
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
copybutton_prompt_text = r'PS >'
copybutton_prompt_is_regexp = True
copybutton_exclude = '.linenos, .gp'

# tabs behavior
sphinx_tabs_disable_tab_closing = True

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
        "ouster_client",
        "ouster_library", 
        "ouster_mapping",
        "ouster_osf",
        "ouster_pcap",
        "ouster_sensor",
        "ouster_viz"
    ])
}
cpp_id_attributes = ['id'] 
cpp_paren_attributes = ['paren']


exhale_args = {
    "containmentFolder": "./cpp/api_cpp",
    "rootFileName": "index.rst",
    "rootFileTitle": "C++ API Reference",
    "pageHierarchySubSectionTitle": "",
    "fullApiSubSectionTitle": "",
    "doxygenStripFromPath": ".",
    "createTreeView": True, 
    "afterTitleDescription": """
    This section provides a comprehensive reference for the Ouster C++ SDK.
    Navigate through the hierarchical structure below to explore classes,namespaces, and files.
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
    "kindsWithContentsDirectives": ["namespace", "class", "struct"],
    "contentsDirectives": True,
    # Use only for debugging:
    # "verboseBuild": False,
    # https://exhale.readthedocs.io/en/latest/reference/configs.html#exhale.configs.generateBreatheFileDirectives
    # "generateBreatheFileDirectives": True,
}

# -----------------------------------------------------------------------------
# HTML theme & assets
# -----------------------------------------------------------------------------
html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "logo_only": True,
    "display_version": True,
    'style_external_links': True,
    # Toc options
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": 4,
    "includehidden": True,
}
html_logo = 'images/Ouster_Logo_TM_Stacked_White_RGB.svg'
html_static_path = ['_static']
html_css_files = [
    'css/ouster_rtd_tweaks.css',
    'css/cpp_autosummary.css',
]
html_js_files = ['js/cpp_autosummary.js',
                 'js/cpp_namespace_toctree.js']

html_context = {
    "display_github": True,
    "github_user": "ouster-lidar",
    "github_repo": "ouster-sdk",
    "github_version": "master",
    "conf_py_path": "/docs/",
    "versions": [["latest", base_url]],
    "current_version": "latest",
}
for v in link_versions:
    html_context["versions"].append([v, f"{base_url}/{v}"])


def skip_deprecated(app, what, name, obj, skip, options):
    deprecated = {'ouster.sdk.osf.multi'}
    return any(d in name for d in deprecated) or skip


# -----------------------------------------------------------------------------
# Post processing for exhale output
# -----------------------------------------------------------------------------
def post_process_exhale_output(app, exception):
    """Move deprecated list from top to bottom of C++ API index page."""
    if exception is not None:
        return
    # Paths
    outdir = Path(app.builder.outdir)
    target = outdir / "cpp" / "api_cpp" / "index.html"
    if not target.exists():
        print("Target file not found")
        return
    html_text = target.read_text(encoding="utf-8")
    # Skip if already processed
    if "<!-- moved-deprecated-block -->" in html_text:
        print("Already processed")
        return
    soup = BeautifulSoup(html_text, "html.parser")
    # Move namespaces in toctree under C++ API Reference
    cpp_api_li = None
    for li in soup.find_all("li", class_="toctree-l1"):
        a = li.find("a", class_="current reference internal")
        if a and a.get_text(strip=True) == "C++ API Reference":
            cpp_api_li = li
            break
    if cpp_api_li:
        # Find the Namespaces <li class="toctree-l2">
        namespaces_li = None
        for li2 in cpp_api_li.find_all("li", class_="toctree-l2"):
            a2 = li2.find("a", class_="reference internal")
            if a2 and a2.get_text(strip=True) == "Namespaces":
                namespaces_li = li2
                break
        if namespaces_li:
            # Find all toctree-l3 items under Namespaces
            l3_items = namespaces_li.find_all("li", class_="toctree-l3", recursive=True)
            # Change their class to toctree-l2 and move them up
            for l3 in l3_items:
                l3['class'] = ['toctree-l2']
                cpp_api_li.ul.append(l3.extract())
            # Remove the Namespaces node
            namespaces_li.decompose()
            print("Flattened Namespaces tree under C++ API Reference")

    # Find and remove deprecated list
    deprecated_ul = soup.find("ul", {"id": "page-treeView", "class": "treeView"})
    if not deprecated_ul:
        print("No deprecated list found")
        return
    print("Found deprecated list - moving to bottom")
    deprecated_html = str(deprecated_ul)
    deprecated_ul.decompose()

    # Create new section at bottom
    article_body = soup.find("div", {"itemprop": "articleBody"})
    if article_body:
        comment = Comment(" moved-deprecated-block ")
        article_body.append(comment)
        # Create deprecated section
        section = soup.new_tag("section", id="deprecated-apis", style="margin-top: 2rem; padding: 1rem; border-top: 2px solid #e1e4e5;")
        h2 = soup.new_tag("h2")
        h2.string = "Deprecated APIs"
        section.append(h2)
        # Parse and add the deprecated content
        deprecated_soup = BeautifulSoup(deprecated_html, "html.parser")
        section.append(deprecated_soup.ul)
        article_body.append(section)
        # Write back
        target.write_text(str(soup), encoding="utf-8")
        print("Successfully moved deprecated list")
    else:
        print("Could not find article body")


def fix_param_direction_html(app, exception):
    if exception is not None:
        return
    outdir = Path(app.builder.outdir) / "cpp" / "api_cpp"
    for html_file in outdir.rglob("*.html"):
        text = html_file.read_text(encoding="utf-8")
        soup = BeautifulSoup(text, "html.parser")
        changed = False
        # Find all <dt class="field-odd"> with "Parameters"
        for dt in soup.find_all("dt", class_="field-odd"):
            if "Parameters" in dt.get_text():
                # Find the next <dd class="field-odd">
                dd = dt.find_next_sibling("dd")
                if dd and dd.get("class") and "field-odd" in dd.get("class"):
                    # Case 1: Parameters as list items
                    # <li><p><strong>param</strong> <strong>[in]</strong> description</p></li>
                    for li in dd.find_all("li"):
                        p = li.find("p")
                        if not p:
                            continue
                        strongs = p.find_all("strong")
                        if len(strongs) >= 2:
                            param_strong = strongs[0]
                            dir_strong = strongs[1]
                            direction = dir_strong.text.strip()
                            param_strong.string = param_strong.text + direction
                            dir_strong.decompose()
                            changed = True
                    # Case 2: Parameters as paragraphs
                    for p in dd.find_all("p", recursive=True):
                        # 2a. <p><strong>param</strong> <strong>[in]</strong> description</p>
                        strongs = p.find_all("strong")
                        if len(strongs) >= 2:
                            param_strong = strongs[0]
                            dir_strong = strongs[1]
                            direction = dir_strong.text.strip()
                            param_strong.string = param_strong.text + direction
                            dir_strong.decompose()
                            changed = True
                        elif len(strongs) >= 1:
                            # 2b: <p><strong>param</strong> – [in] description</p>
                            strong = strongs[0]
                            siblings = list(strong.next_siblings)
                            for sib in siblings:
                                if isinstance(sib, str):
                                    # Match pattern: " – [in]" or "– [in]"
                                    import re
                                    m = re.match(r"\s*–\s*(\[[^\]]+\])", sib)
                                    if m and any(x in m.group(1) for x in ["in", "out"]):
                                        direction = m.group(1)
                                        strong.string = strong.text + direction
                                        # Remove the direction from the text node
                                        new_text = sib.replace(f"– {direction}", "").replace(f" – {direction}", "")
                                        sib.replace_with(new_text)
                                        changed = True
                        else:
                            print(f"[fix_param_direction_html] warning: no <strong> found in paragraph in {html_file}")
        if changed:
            html_file.write_text(str(soup), encoding="utf-8")

def setup(app):
    app.connect('build-finished', fix_param_direction_html)
    app.connect("build-finished", post_process_exhale_output)
    app.connect("autodoc-skip-member", skip_deprecated)
