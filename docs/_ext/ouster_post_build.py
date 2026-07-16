"""
Sphinx extension Ouster SDK documentation post-processing.
"""
from __future__ import annotations

import copy
import importlib
import inspect
import posixpath
import re
import traceback
from pathlib import Path
from typing import Optional

from bs4 import BeautifulSoup, Comment, NavigableString, Tag
from sphinx.application import Sphinx
from sphinx.util import logging

# autodoc-process-docstring is added by sphinx.ext.autodoc (not core Sphinx).
# See: https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html
from nanobind import process_docstring_for_autodoc
from python_module_ast import PublicExport, get_package_analysis


class PostBuildManager:
    """Encapsulates all "post-build" mutations"""

    _SIDEBAR_TOGGLE_ARROW_SVG = """
    <svg viewBox="0 0 24 24" aria-hidden="true" focusable="false" class="sidebar-arrow">
    <path d="M8 4l8 8-8 8" fill="none" stroke="currentColor" stroke-width="2"
          stroke-linecap="round" stroke-linejoin="round"></path>
    </svg>
    """

    # Placed after std in the injected ouster::sdk namespace list.
    _CPP_DEFERRED_AFTER_STD_HREFS = (
        "namespace_Eigen.html",
        "namespace_ouster__sdk__core__image.html",
    )

    _DEPRECATED_AUTODOC_TARGETS = {"ouster.sdk.osf.multi"}

    _PYTHON_API_PINNED_TITLES = frozenset({
        "module contents",
        "subpackages",
        "submodules",
    })
    _PYTHON_API_GROUP_TITLES = frozenset({"classes", "functions", "submodules"})

    _CPP_API_SECTION_ORDER = (
        "namespaces", "classes", "functions", "enums", "typedefs",
        "variables", "defines", "dirs", "files",
    )
    _CPP_LINK_LABEL_PREFIX_RE = re.compile(
        r"^(?:Namespace\s+|Class|Struct|Function|Typedef|Variable|Enum|Union|Define|Dir|File)\s+",
        re.IGNORECASE,
    )

    def __init__(self, app: Sphinx) -> None:
        self.app = app
        self.docs_dir = Path(app.confdir).resolve()
        self._cpp_namespace_root: Optional[BeautifulSoup] = None
        self._cpp_namespace_member_groups: dict[str, dict[str, dict]] = {}
        self._python_symbol_kinds_cache: dict[str, dict[str, str]] = {}
        self.exhale_cpp_dir, self.exhale_api_dir = self._compute_exhale_parts()
        self.logger = logging.getLogger(__name__)

    # ------------------------ setup ------------------------------------------
    def register(self) -> None:
        """Connect all the Sphinx hooks."""
        app = self.app
        app.connect(
            "build-finished",
            self.capture_cpp_namespace_sidebar,
            priority=100,
        )
        app.connect("build-finished", self.rewrite_bindings_links)
        app.connect("build-finished", self.fix_param_direction_html)
        app.connect("build-finished", self.post_process_exhale_output)
        app.connect("build-finished", self.overwrite_collapsible_lists_js)
        app.connect("build-finished", self.inject_sidebar_for_modules)
        app.connect("build-finished", self.build_api_left_tables)
        app.connect("autodoc-skip-member", self.skip_deprecated)
        app.connect("autodoc-process-docstring", process_docstring_for_autodoc)

    # -------------------------- utilities ------------------------------------
    def _compute_exhale_parts(self) -> tuple[str, str]:
        """Derive <cpp dir>/<api dir> from the configured containment folder."""
        args = getattr(self.app.config, "exhale_args", {}) or {}
        folder = args.get("containmentFolder", "./cpp/api_cpp")
        folder = folder.strip("./")
        parts = [part for part in folder.split("/") if part]
        if len(parts) >= 2:
            return parts[0], parts[1]
        if len(parts) == 1:
            return parts[0], "api_cpp"
        return "cpp", "api_cpp"

    def _api_outdir(self) -> Path:
        return Path(self.app.builder.outdir) / self.exhale_cpp_dir / self.exhale_api_dir

    def _read_html_soup(self, html_path: Path) -> BeautifulSoup:
        return BeautifulSoup(html_path.read_text(encoding="utf-8"), "html.parser")

    def _write_html_soup_if_changed(
        self, html_path: Path, soup: BeautifulSoup, changed: bool
    ) -> bool:
        if not changed:
            return False
        html_path.write_text(str(soup), encoding="utf-8")
        return True

    # --------------------------- event hooks -----------------------------------------------
    def skip_deprecated(self, _app, _what, name, _obj, skip, _options):
        return any(d in name for d in self._DEPRECATED_AUTODOC_TARGETS) or skip

    def overwrite_collapsible_lists_js(self, app: Sphinx, exception) -> None:
        if exception is not None:
            return
        outdir = Path(app.builder.outdir)
        destination = outdir / "_static" / "collapsible-lists" / "js" / "apply-collapsible-lists.js"
        source = self.docs_dir / "_static" / "collapsible-lists" / "js" / "apply-collapsible-lists.js"
        if source.exists():
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
            self.logger.info("Overrode collapsible lists script with jQuery-free version")

    def rewrite_bindings_links(self, app: Sphinx, exception) -> None:
        if exception:
            return
        outdir = Path(app.builder.outdir) / 'python' / 'api_python'
        if not outdir.exists():
            return
        for html_path in outdir.rglob('*.html'):
            text = html_path.read_text(encoding='utf-8')
            if 'ouster.sdk._bindings.' not in text:
                continue
            if 'ouster.sdk._bindings.client' in text:
                text = text.replace('ouster.sdk._bindings.client', 'ouster.sdk.core')
            text = text.replace('ouster.sdk._bindings.', 'ouster.sdk.')
            html_path.write_text(text, encoding='utf-8')

    def fix_param_direction_html(self, app: Sphinx, exception) -> None:
        if exception is not None:
            return
        outdir = self._api_outdir()
        for html_file in outdir.rglob("*.html"):
            soup = self._read_html_soup(html_file)
            changed = False
            # Find all <dt class="field-odd"> with "Parameters"
            for dt in soup.select("dt.field-odd, dt.field-even"):
                if "Parameters" in dt.get_text():
                    # Find the next <dd class="field-odd">
                    dd = dt.find_next_sibling("dd")
                    if not isinstance(dd, Tag):
                        continue
                    dd_classes = dd.get("class")
                    if not dd_classes or not any(
                        cls in {"field-odd", "field-even"} for cls in dd_classes
                    ):
                        continue

                    def _remove_dash_before(node):
                        prev = node.previous_sibling
                        if prev and isinstance(prev, NavigableString):
                            text = prev.strip()
                            if text in {"–", "-"} or text.startswith("–") or text.startswith("-"):
                                prev.extract()

                    # helper to combine param + direction and drop dash text
                    def _append_direction(param_strong, dir_strong):
                        direction = dir_strong.text.strip()
                        param_strong.string = param_strong.text + direction
                        _remove_dash_before(dir_strong)
                        dir_strong.decompose()
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
                            _append_direction(param_strong, dir_strong)
                            changed = True
                    # Case 2: Parameters as paragraphs
                    for p in dd.find_all("p", recursive=True):
                        strongs = p.find_all("strong")
                        if len(strongs) >= 2:
                            param_strong = strongs[0]
                            dir_strong = strongs[1]
                            _append_direction(param_strong, dir_strong)
                            changed = True
                        elif len(strongs) >= 1:
                            # 2b: <p><strong>param</strong> – [in] description</p>
                            strong = strongs[0]
                            siblings = list(strong.next_siblings)
                            for sib in siblings:
                                if isinstance(sib, NavigableString):
                                    text = str(sib)
                                    m = re.match(r"\s*–\s*(\[[^\]]+\])", text)
                                    if m and any(x in m.group(1) for x in ["in", "out"]):
                                        direction = m.group(1)
                                        strong.string = strong.text + direction
                                        new_text = text.replace(f"– {direction}", "").replace(f" – {direction}", "")
                                        sib.replace_with(new_text)
                                        changed = True
                        else:
                            self.logger.info(
                                "[fix_param_direction_html] no <strong> found in "
                                f"paragraph in {html_file}"
                            )
            self._write_html_soup_if_changed(html_file, soup, changed)

    def post_process_exhale_output(self, app: Sphinx, exception) -> None:
        """Move deprecated list from top to bottom of C++ API index page."""
        if exception is not None:
            return
        # Paths
        outdir = Path(app.builder.outdir)

        root_file = getattr(app.config, "exhale_args", {}).get("rootFileName", "index.rst")
        target_name = Path(root_file).with_suffix(".html").name
        target = outdir / self.exhale_cpp_dir / self.exhale_api_dir / target_name
        if not target.exists():
            self.logger.warning("Target file not found, exhale maybe disabled")
            return
        html_text = target.read_text(encoding="utf-8")
        # Skip if already processed
        if "moved-deprecated-block" in html_text:
            self.logger.info("Already processed")
            return
        soup = BeautifulSoup(html_text, "html.parser")

        # Find and remove deprecated list
        deprecated_ul = soup.find("ul", {"id": "page-treeView", "class": "treeView"})
        if not isinstance(deprecated_ul, Tag):
            self.logger.warning("No deprecated list found")
            return
        self.logger.info("Found deprecated list - moving to bottom")
        deprecated_html = str(deprecated_ul)
        deprecated_ul.decompose()

        # Create new section at bottom
        article_body = soup.find("div", {"itemprop": "articleBody"})
        if article_body:
            comment = Comment(" moved-deprecated-block ")
            article_body.append(comment)
            # Create deprecated section
            section = soup.new_tag(
                "section",
                id="deprecated-apis",
                style="margin-top: 2rem; padding: 1rem; border-top: 2px solid #e1e4e5;",
            )
            h2 = soup.new_tag("h2")
            h2.string = "Deprecated APIs"
            section.append(h2)
            # Parse and add the deprecated content
            deprecated_soup = BeautifulSoup(deprecated_html, "html.parser")
            deprecated_ul_tag = deprecated_soup.ul
            if deprecated_ul_tag is not None:
                section.append(deprecated_ul_tag)
            article_body.append(section)
            # Write back
            target.write_text(str(soup), encoding="utf-8")
            self.logger.info("Successfully moved deprecated list")
        else:
            self.logger.warning("Could not find article body")
            # Change to exit later, sys.exit(1)

        # Remove redundant namespace toctree.
        namespace_page = outdir / self.exhale_cpp_dir / self.exhale_api_dir / "namespace_ouster.html"
        if namespace_page.exists():
            namespace_html = namespace_page.read_text(encoding="utf-8")
            namespace_soup = BeautifulSoup(namespace_html, "html.parser")
            removed = False
            removed_ids = []
            for section in namespace_soup.find_all("section"):
                if not isinstance(section, Tag):
                    continue
                section_id_raw = section.get("id", "")
                section_id = section_id_raw if isinstance(section_id_raw, str) else ""
                if section_id.startswith("id") and section.find("div", class_="toctree-wrapper"):
                    section.decompose()
                    removed_ids.append(section_id)
                    removed = True
            nav_tag = namespace_soup.find("nav", class_="right-toc")
            nav = nav_tag if isinstance(nav_tag, Tag) else None
            if removed and nav is not None:
                for section_id in removed_ids:
                    for anchor in nav.select(f'a[href="#{section_id}"]'):
                        li = anchor.find_parent("li")
                        if li:
                            li.decompose()
            namespaces_section = namespace_soup.find("section", {"id": "namespaces"})
            if isinstance(namespaces_section, Tag):
                self.logger.info("Removing duplicate namespaces list section from namespace_ouster.html")
                namespaces_section.decompose()
            # Remove trailing Namespaces section
            bottom_ns_section = None
            for section in namespace_soup.find_all("section"):
                if not isinstance(section, Tag):
                    continue
                header = section.find(["h2", "h3"])
                if header and header.get_text(strip=True) == "Namespaces":
                    bottom_ns_section = section
                    break
            if isinstance(bottom_ns_section, Tag):
                self.logger.info("Removing trailing Namespaces section from namespace_ouster.html")
                bottom_section_id_raw = bottom_ns_section.get("id")
                bottom_section_id = (
                    bottom_section_id_raw if isinstance(bottom_section_id_raw, str) else None
                )
                bottom_ns_section.decompose()
                if bottom_section_id and nav is not None:
                    for anchor in nav.select(f'a[href="#{bottom_section_id}"]'):
                        li = anchor.find_parent("li")
                        if li:
                            li.decompose()
            namespace_page.write_text(str(namespace_soup), encoding="utf-8")
            self.logger.info("Removed duplicate namespace toctree from namespace_ouster.html")

        # Remove parentheses around inline code + reference pairs in Exhale pages
        api_root = outdir / self.exhale_cpp_dir / self.exhale_api_dir
        if api_root.exists():
            for html_file in api_root.rglob("*.html"):
                soup = self._read_html_soup(html_file)
                updated = False
                for paragraph in soup.find_all("p"):
                    code_tag = paragraph.find("code", class_="docutils literal notranslate")
                    link_tag = paragraph.find("a", class_="reference internal")
                    if not code_tag or not link_tag:
                        continue
                    prev_sib = link_tag.previous_sibling
                    next_sib = link_tag.next_sibling
                    removed = False
                    if isinstance(prev_sib, NavigableString):
                        prev_text = str(prev_sib)
                        new_prev = re.sub(r"\s*\($", "", prev_text)
                        if new_prev != prev_text:
                            if new_prev:
                                prev_sib.replace_with(new_prev)
                            else:
                                prev_sib.extract()
                            removed = True
                    if isinstance(next_sib, NavigableString):
                        next_text = str(next_sib)
                        new_next = re.sub(r"^\)\s*", "", next_text)
                        if new_next != next_text:
                            if new_next:
                                next_sib.replace_with(new_next)
                            else:
                                next_sib.extract()
                            removed = True
                    if removed:
                        updated = True
                if self._write_html_soup_if_changed(html_file, soup, updated):
                    rel_path = html_file.relative_to(outdir)
                    self.logger.info(f"Removed inline code parentheses in {rel_path}")

    def capture_cpp_namespace_sidebar(self, app: Sphinx, exception) -> None:
        """Capture the raw namespace sidebar before we update pages."""
        if exception is not None:
            return

        outdir = Path(app.builder.outdir)
        ns_path = outdir / self.exhale_cpp_dir / self.exhale_api_dir / "namespace_ouster.html"
        if not ns_path.exists():
            self.logger.debug("namespace_ouster.html missing; cannot cache nav")
            return

        # Read the file content
        try:
            soup = BeautifulSoup(ns_path.read_text(encoding="utf-8"), "html.parser")
        except Exception as e:
            self.logger.warning(f"Failed to read namespace_ouster.html: {e}")
            return

        wrappers = soup.select("div.toctree-wrapper.compound")
        collected = []
        for wrapper in wrappers:
            ul = wrapper.find("ul")
            if not isinstance(ul, Tag):
                continue
            for li in ul.find_all("li", class_="toctree-l1", recursive=False):
                collected.append(li)

        if not collected:
            self.logger.debug("namespace page missing toctree wrappers; cannot cache nav")
            return

        factory = BeautifulSoup("", "html.parser")
        root_li = factory.new_tag("li")
        root_li["class"] = "toctree-l1"
        root_div = factory.new_tag("div")
        root_div["class"] = "sidebar-link-row"
        root_anchor = factory.new_tag("a", href="namespace_ouster__sdk.html")
        root_anchor["class"] = "reference internal"
        root_anchor.string = "Namespace ouster::sdk"
        root_div.append(root_anchor)
        root_li.append(root_div)

        def _bump_toctree_levels(li_tag, delta=1):
            if li_tag is None:
                return
            targets = [li_tag] + li_tag.find_all("li")
            for node in targets:
                classes = node.get("class", []) or []
                new_classes = []
                for cls in classes:
                    if cls.startswith("toctree-l"):
                        try:
                            level = int(cls.replace("toctree-l", ""))
                            new_classes.append(f"toctree-l{level + delta}")
                        except ValueError:
                            new_classes.append(cls)
                    else:
                        new_classes.append(cls)
                if new_classes:
                    node["class"] = new_classes

        def _prune_chanfield_entries(parent):
            if parent is None:
                return
            for li in list(parent.find_all("li")):
                link = li.find("a", class_="reference")
                label = link.get_text(strip=True) if link else ""
                if label and "ChanField" in label:
                    li.decompose()

        nav_ul = factory.new_tag("ul")
        nav_ul["class"] = "current"
        for li in collected:
            link = li.find("a", class_="reference internal")
            if not link:
                continue
            label = link.get_text(strip=True)
            if not label or "ChanField" in label:
                continue
            cloned_li = BeautifulSoup(str(li), "html.parser").find("li")
            if cloned_li is None:
                continue
            _prune_chanfield_entries(cloned_li)
            _bump_toctree_levels(cloned_li, delta=1)
            nav_ul.append(cloned_li)

        if not nav_ul.contents:
            self.logger.debug("namespace wrappers did not yield any entries; cannot cache nav")
            return

        root_li.append(nav_ul)
        self._reorder_cpp_namespace_nav(root_li)

        prepared_root = self._prepare_cpp_namespace_root(root_li)
        if prepared_root is None:
            self.logger.warning("failed to prepare namespace nav root")
            return

        self._cpp_namespace_root = prepared_root
        self.logger.debug(f"cached namespace nav for later injection: {prepared_root}")

    def inject_sidebar_for_modules(self, app: Sphinx, exception) -> None:
        """Clone the python API sidebar into generated _modules pages post-build."""
        if exception is not None:
            return

        outdir = Path(app.builder.outdir)
        modules_root = outdir / "_modules"
        if not modules_root.exists():
            self.logger.debug("_inject_sidebar_for_modules: no _modules directory, skipping")
            return

        def _find_source_sidebar(module_html: Path):
            rel_parts = module_html.relative_to(outdir).parts
            if len(rel_parts) <= 1:
                return None, None

            module_parts = list(rel_parts[1:])
            stem = module_parts[-1][:-5]  # strip ".html"
            module_parts = module_parts[:-1] + [stem]
            if not module_parts:
                return None, None

            dotted = ".".join(module_parts)
            python_rel = Path("python") / "api_python" / f"{dotted}.html"
            python_file = outdir / python_rel
            if not python_file.exists():
                return None, None

            python_html = BeautifulSoup(python_file.read_text(encoding="utf-8"), "html.parser")
            sidebar = (
                python_html.select_one(".vp-sidebar .sidebar-links")
                or python_html.select_one(".sidebar-links")
            )
            if sidebar is None:
                return None, None
            return sidebar, python_rel.as_posix()

        def _rebase_links(sidebar: Tag, src_dir: str, dst_dir: str) -> None:
            for anchor in sidebar.find_all("a", href=True):
                href_attr = anchor.get("href")
                if not isinstance(href_attr, str):
                    continue
                href = href_attr
                if not href or href.startswith(("#", "http://", "https://")):
                    continue
                target = posixpath.normpath(posixpath.join(src_dir, href))
                rebased = posixpath.relpath(target, dst_dir)
                anchor["href"] = rebased

        def _ensure_class(tag, classname: str) -> None:
            classes = tag.get("class") or []
            if classname not in classes:
                classes.append(classname)
                tag["class"] = classes

        def _mark_default_entry(sidebar: Tag) -> None:
            anchor = None
            for candidate in sidebar.find_all("a", href=True):
                href_attr = candidate.get("href")
                if not isinstance(href_attr, str):
                    continue
                if href_attr.endswith("ouster.sdk.html"):
                    anchor = candidate
                    break
            if not anchor:
                return

            li = anchor.find_parent("li")
            while li:
                _ensure_class(li, "current")
                toggle = li.find("button", class_="sidebar-toggle")
                if toggle:
                    _ensure_class(toggle, "is-open")
                    toggle["aria-expanded"] = "true"
                parent_ul = li.find_parent("ul")
                if parent_ul:
                    _ensure_class(parent_ul, "current")
                li = li.find_parent("li")

        for module_html in modules_root.rglob("*.html"):
            sidebar, python_rel = _find_source_sidebar(module_html)
            if sidebar is None or python_rel is None:
                continue

            module_rel = module_html.relative_to(outdir).as_posix()
            python_dir = posixpath.dirname(python_rel)
            module_dir = posixpath.dirname(module_rel)

            sidebar_soup = BeautifulSoup(sidebar.decode(), "html.parser")
            sidebar_copy = sidebar_soup.select_one(".sidebar-links")
            if sidebar_copy is None:
                continue
            _rebase_links(sidebar_copy, python_dir, module_dir)
            self._normalize_python_module_labels(sidebar_copy, module_rel)
            _mark_default_entry(sidebar_copy)

            module_soup = BeautifulSoup(module_html.read_text(encoding="utf-8"), "html.parser")
            target_sidebar = (
                module_soup.select_one(".vp-sidebar .sidebar-links")
                or module_soup.select_one(".sidebar-links")
            )
            if target_sidebar is None:
                continue
            target_sidebar.replace_with(sidebar_copy)
            module_html.write_text(str(module_soup), encoding="utf-8")

    def build_api_left_tables(self, app: Sphinx, exception) -> None:
        self.logger.debug("Starting API sidebar injection")
        if exception:
            return
        outdir = Path(app.builder.outdir)
        cpp_root = outdir / "cpp" / "api_cpp"
        self._cpp_namespace_member_groups.clear()
        if cpp_root.exists():
            for html_path in sorted(cpp_root.glob("namespace_*.html")):
                try:
                    soup = self._read_html_soup(html_path)
                    groups = self._extract_cpp_namespace_member_groups(soup)
                    if groups:
                        self._cpp_namespace_member_groups[html_path.name] = groups
                except Exception as exc:
                    self.logger.warning(
                        f"Failed to cache C++ namespace members for {html_path}: {exc}"
                    )
                    continue

        for rel in ["python/api_python", "cpp/api_cpp"]:
            root = outdir / rel
            if not root.exists():
                continue
            for html_path in root.rglob("*.html"):
                try:
                    self._inject_left_api_sections(html_path)
                except Exception as exc:
                    self.logger.warning(f"Failed to update API sidebar for {html_path}: {exc}")
                    raise

    # ---------------- helper methods ----------------------------------------
    def _inject_left_api_sections(self, html_path: Path) -> None:
        normalized_rel = str(html_path).replace("\\", "/")
        soup = self._read_html_soup(html_path)
        sidebar = soup.select_one(".sidebar-links")
        if sidebar is None:
            self.logger.debug(f"{normalized_rel}: missing .sidebar-links; skipping")
            return

        heading_nodes = self._extract_heading_nodes_for_api(soup)

        self._remove_existing_api_sections(sidebar)

        is_python_page = "python/api_python/" in normalized_rel
        is_cpp_page = "cpp/api_cpp/" in normalized_rel
        if is_cpp_page:
            self._adjust_cpp_namespace_tree(sidebar, normalized_rel, soup)

        def _write_with_normalization():
            if is_python_page:
                self._normalize_python_module_labels(sidebar, normalized_rel)
                self.logger.debug(f"{normalized_rel}: normalized python module labels")
            elif is_cpp_page:
                self._normalize_cpp_namespace_labels(sidebar, normalized_rel)
                self.logger.debug(f"{normalized_rel}: normalized cpp namespace labels")
            html_path.write_text(str(soup), encoding="utf-8")
            self.logger.debug(f"{normalized_rel}: wrote sidebar updates")

        cpp_ns_href = (
            self._cpp_namespace_href_for_page(html_path, soup) if is_cpp_page else None
        )
        has_cpp_member_groups = bool(
            cpp_ns_href and cpp_ns_href in self._cpp_namespace_member_groups
        )
        if len(heading_nodes) <= 1 and not (is_python_page or is_cpp_page or has_cpp_member_groups):
            _write_with_normalization()
            return

        def _extract_level(tag):
            classes = tag.get("class", []) or []
            if isinstance(classes, str):
                classes = classes.split()
            for cls in classes:
                if cls.startswith("toctree-l"):
                    try:
                        return int(cls.split("toctree-l", 1)[1])
                    except ValueError:
                        continue
            return None

        current_li = None
        if is_cpp_page:
            current_li = self._find_cpp_namespace_li(sidebar, html_path, soup)
        if current_li is None:
            active_link = sidebar.select_one('a[aria-current="page"]')
            if active_link:
                current_li = active_link.find_parent("li")
        if current_li is None:
            deepest_li = None
            deepest_level = -1
            for li in sidebar.select("li"):
                classes_attr = li.attrs.get("class")
                classes: list[str]
                if isinstance(classes_attr, str):
                    classes = classes_attr.split()
                elif classes_attr:
                    classes = list(classes_attr)
                else:
                    classes = []
                if "current" not in classes:
                    continue
                level = _extract_level(li)
                if level is None:
                    continue
                if level > deepest_level:
                    deepest_level = level
                    deepest_li = li
            if deepest_li is not None:
                current_li = deepest_li
        if current_li is None:
            current_li = sidebar.select_one("li.toctree-l1.current")
        elif not is_cpp_page:
            level = _extract_level(current_li)
            if level == 1:
                nested_current = [
                    link.find_parent("li")
                    for link in current_li.select("a.reference.internal.current")
                    if link.find_parent("li") is not None
                ]
                nested_current = [li for li in nested_current if _extract_level(li)]
                if nested_current:
                    nested_current.sort(key=lambda li: _extract_level(li), reverse=True)
                    current_li = nested_current[0]
        if current_li is None:
            _write_with_normalization()
            return
        else:
            label_anchor = current_li.find("a", class_="reference")
            label = label_anchor.get_text(strip=True) if label_anchor else current_li.get_text(strip=True)

        sections_ul = current_li.select_one("ul.api-section-list")
        if sections_ul:
            sections_ul.clear()
        else:
            sections_ul = soup.new_tag("ul")
            sections_ul["class"] = "api-section-list"
            current_li.append(sections_ul)

        if is_python_page:
            page_module = html_path.stem.replace("_", ".")
            self._populate_python_api_section_list(
                soup, sections_ul, heading_nodes, page_module)
        elif is_cpp_page:
            if has_cpp_member_groups and cpp_ns_href is not None:
                self._populate_cpp_api_section_list(
                    sections_ul, soup, cpp_ns_href, html_path.name)
            else:
                self._populate_cpp_api_section_list_from_headings(
                    sections_ul, soup, heading_nodes)
        elif has_cpp_member_groups and cpp_ns_href is not None:
            self._populate_cpp_api_section_list(
                sections_ul, soup, cpp_ns_href, html_path.name)
        else:
            self._populate_cpp_api_section_list_from_headings(
                sections_ul, soup, heading_nodes)

        # C++ namespace pages can occasionally miss heading extraction if the
        # page structure varies; ensure standard section links are still present.
        if is_cpp_page and not sections_ul.find("li", recursive=False):
            for section_id, label in (
                ("namespaces", "Namespaces"),
                ("classes", "Classes"),
                ("functions", "Functions"),
                ("enums", "Enums"),
                ("typedefs", "Typedefs"),
                ("variables", "Variables"),
                ("defines", "Defines"),
                ("dirs", "Dirs"),
                ("files", "Files"),
            ):
                if soup.find(id=section_id) is None:
                    continue
                link = soup.new_tag("a", href=f"#{section_id}")
                link.string = label
                link["data-cpp-nav-clean"] = "1"
                item = soup.new_tag("li")
                item.append(link)
                sections_ul.append(item)

        top_sections = sections_ul.find_all("li", recursive=False)
        if not top_sections:
            sections_ul.decompose()
        else:
            if is_cpp_page or is_python_page:
                self._collapse_api_section_groups(
                    sections_ul, html_path.name, is_cpp_page=is_cpp_page)
            if is_cpp_page:
                self._remove_cpp_duplicate_section_ul(current_li)
            if sections_ul and sections_ul.find("li", recursive=False):
                current_level = _extract_level(current_li)
                if (is_python_page or is_cpp_page) and current_level == 1:
                    # l1 toggle controls the package/namespace tree, not
                    # api-section-list; keep both open on landing pages.
                    self._ensure_sidebar_toggle(current_li, is_open=True)
                    self._set_nav_ul_open(sections_ul, is_open=True)
                else:
                    self._ensure_sidebar_toggle(
                        current_li, is_open=True, nav_ul=sections_ul)
            self.logger.debug(f"{normalized_rel}: built {len(top_sections)} api section entries")

        _write_with_normalization()

    _CPP_EXHALE_FILE_PREFIXES = (
        "struct", "class", "enum", "union", "function", "variable",
        "typedef", "dir", "file", "define",
    )

    @classmethod
    def _clean_cpp_sidebar_label(cls, raw_label: str) -> str:
        label = raw_label.strip()
        if not label:
            return label
        return cls._CPP_LINK_LABEL_PREFIX_RE.sub("", label).strip() or label

    @staticmethod
    def _heading_anchor_id(header) -> str:
        anchor_id = (header.get("id") or "").strip()
        if anchor_id:
            return anchor_id
        parent_section = header.find_parent("section")
        if parent_section and parent_section.get("id"):
            return (parent_section.get("id") or "").strip()
        header_link = header.find("a", class_="headerlink")
        if header_link:
            href = header_link.get("href")
            if href and href.startswith("#"):
                return href.lstrip("#")
        return ""

    @staticmethod
    def _heading_title_text(header) -> str:
        title = header.get_text(" ", strip=True)
        header_link = header.find("a", class_="headerlink")
        if header_link:
            marker = header_link.get_text(" ", strip=True)
            if marker:
                title = title.replace(marker, "").strip()
        return title

    def _extract_cpp_namespace_member_groups(self, soup) -> dict[str, dict]:
        """Parse Exhale namespace summary lists into sidebar group data."""
        content = soup.select_one(".vp-doc") or soup.select_one(".content")
        if content is None:
            return {}

        groups: dict[str, dict] = {}
        for h2 in content.find_all("h2"):
            anchor_id = self._heading_anchor_id(h2)
            if not anchor_id:
                continue
            title = self._heading_title_text(h2)
            if not title:
                continue
            ul = h2.find_next_sibling("ul")
            if ul is None:
                continue

            members: list[dict] = []
            for link in ul.select("li a.reference.internal[href]"):
                href = (link.get("href") or "").strip()
                if not href:
                    continue
                label = self._clean_cpp_sidebar_label(link.get_text(" ", strip=True))
                if not label:
                    continue
                members.append({
                    "href": href,
                    "id": href.split("#", 1)[0],
                    "label": label,
                    "title": label,
                })

            if members:
                groups[anchor_id] = {"label": title, "members": members}
        return groups

    @staticmethod
    def _remove_cpp_duplicate_section_ul(current_li) -> None:
        """Drop Exhale toctree-l3 section links superseded by api-section-list."""
        if current_li is None:
            return
        li_classes = current_li.get("class", []) or []
        if isinstance(li_classes, str):
            li_classes = li_classes.split()
        if "toctree-l2" not in li_classes:
            return
        for child_ul in list(current_li.find_all("ul", recursive=False)):
            classes = child_ul.get("class", []) or []
            if isinstance(classes, str):
                classes = classes.split()
            if "api-section-list" in classes:
                continue
            child_ul.decompose()

    def _populate_cpp_api_section_list(
            self,
            sections_ul,
            soup,
            ns_href: str,
            current_page_basename: str,
    ) -> None:
        cached = self._cpp_namespace_member_groups.get(ns_href)
        if not cached:
            return

        namespace_label_prefixes = self._cpp_namespace_label_prefixes(ns_href)

        def _strip_namespace_prefix(node: dict) -> dict:
            if not namespace_label_prefixes:
                return node
            label = node.get("label", "")
            for prefix in namespace_label_prefixes:
                marker = f"{prefix}::"
                if not label.startswith(marker):
                    continue
                trimmed = dict(node)
                trimmed["label"] = label[len(marker):]
                return trimmed
            return node

        seen: set[str] = set()
        for anchor_id in self._CPP_API_SECTION_ORDER:
            group = cached.get(anchor_id)
            if group is None:
                continue
            seen.add(anchor_id)
            members = [_strip_namespace_prefix(member) for member in group["members"]]
            self._append_api_section_group(
                sections_ul, soup, anchor_id, group["label"], members)

        for anchor_id, group in cached.items():
            if anchor_id in seen:
                continue
            members = [_strip_namespace_prefix(member) for member in group["members"]]
            self._append_api_section_group(
                sections_ul, soup, anchor_id, group["label"], members)

        self._mark_cpp_sidebar_current_member(sections_ul, current_page_basename)

    @staticmethod
    def _mark_cpp_sidebar_current_member(sections_ul, current_page_basename: str) -> None:
        for link in sections_ul.select("a[href]"):
            href = (link.get("href") or "").split("#", 1)[0]
            if not href or href.startswith("#"):
                continue
            href_base = href.rsplit("/", 1)[-1]
            if href_base != current_page_basename:
                continue
            li = link.find_parent("li")
            if li is None:
                continue
            li_classes = list(li.get("class", []) or [])
            if "current" not in li_classes:
                li_classes.append("current")
                li["class"] = li_classes
            link_classes = list(link.get("class", []) or [])
            if "current" not in link_classes:
                link_classes.append("current")
                link["class"] = link_classes

    @staticmethod
    def _cpp_namespace_label_prefix(ns_href: str) -> str:
        """Convert namespace_ouster__sdk__core.html -> core."""
        if not ns_href.startswith("namespace_") or not ns_href.endswith(".html"):
            return ""
        namespace_tokens = ns_href[len("namespace_"):-len(".html")].split("__")
        if namespace_tokens[:2] == ["ouster", "sdk"]:
            namespace_tokens = namespace_tokens[2:]
        return "::".join(token for token in namespace_tokens if token)

    @staticmethod
    def _cpp_namespace_label_prefixes(ns_href: str) -> tuple[str, ...]:
        """Return possible namespace prefixes ordered by specificity."""
        if not ns_href.startswith("namespace_") or not ns_href.endswith(".html"):
            return ()
        tokens = [token for token in ns_href[len("namespace_"):-len(".html")].split("__") if token]
        if not tokens:
            return ()
        full = "::".join(tokens)
        prefixes = [full]
        if tokens[:2] == ["ouster", "sdk"] and len(tokens) > 2:
            short = "::".join(tokens[2:])
            if short and short != full:
                prefixes.append(short)
        return tuple(prefixes)

    @staticmethod
    def _populate_cpp_api_section_list_from_headings(
            sections_ul, soup, heading_nodes: list) -> None:
        for node in heading_nodes:
            if node["level"] != 2:
                continue
            node_id = node.get("id")
            if not node_id:
                continue
            link = soup.new_tag("a", href=f"#{node_id}")
            link.string = node["title"]
            link["data-cpp-nav-clean"] = "1"
            section_li = soup.new_tag("li")
            section_li.append(link)
            sections_ul.append(section_li)

    @staticmethod
    def _collapse_api_section_groups(
            sections_ul,
            current_page_basename: str,
            is_cpp_page: bool,
    ) -> None:
        """Pre-collapse nested api-section-list groups; JS expands active ones."""
        for group_li in sections_ul.find_all("li", recursive=False):
            sub_ul = group_li.find("ul", recursive=False)
            if sub_ul is None:
                continue
            should_open = False
            if is_cpp_page:
                for link in sub_ul.select("a[href]"):
                    href = (link.get("href") or "").split("#", 1)[0]
                    if href and href.rsplit("/", 1)[-1] == current_page_basename:
                        should_open = True
                        break
            if should_open:
                continue
            classes = list(sub_ul.get("class", []) or [])
            if "collapsed" not in classes:
                classes.append("collapsed")
                sub_ul["class"] = classes

    @staticmethod
    def _sidebar_li_basename(li_tag) -> str:
        """Basename of a sidebar row's primary link (no URL fragment)."""
        link = li_tag.find("a", class_="reference internal")
        if not link:
            return ""
        href = link.get("href") or ""
        return href.split("#", 1)[0].rsplit("/", 1)[-1]

    def _reorder_cpp_namespace_nav(self, root_tag) -> None:
        """Place std, then Eigen and core::image, after other ouster::sdk namespaces."""
        std_href = "namespace_std.html"
        deferred = self._CPP_DEFERRED_AFTER_STD_HREFS
        tail_hrefs = frozenset({"page_deprecated.html", ""})
        markers = {std_href, *deferred}

        def _sort_key(li):
            href = self._sidebar_li_basename(li)
            if href in tail_hrefs:
                return (3, 0)
            if href == std_href:
                return (1, 0)
            if href in deferred:
                return (2, deferred.index(href))
            return (0, 0)

        for ul in root_tag.find_all("ul"):
            children = ul.find_all("li", recursive=False)
            if len(children) < 2:
                continue
            hrefs = {self._sidebar_li_basename(li) for li in children}
            if not hrefs.intersection(markers):
                continue
            ordered = sorted(children, key=_sort_key)
            if ordered == children:
                continue
            for li in ordered:
                ul.append(li)

    def _cpp_namespace_href_for_page(self, html_path: Path, page_soup=None) -> Optional[str]:
        """Map an Exhale HTML page to its parent namespace_*.html filename."""
        name = html_path.name
        if name.startswith("namespace_") and name.endswith(".html"):
            return name

        stem = html_path.stem
        candidates: list[str] = []
        for candidate in (
            self._cpp_namespace_href_from_munged_stem(stem),
            self._cpp_namespace_href_from_group_stem(stem),
        ):
            if candidate:
                candidates.append(candidate)

        if page_soup is not None:
            # Infer owner namespace from rendered C++ signature prefix.
            # This handles Exhale member pages like function_*_8h_1*.html and
            # variable_*_8h_1*.html where the filename does not encode namespace.
            #  _8h → .h
            # _8hpp → .hpp
            # _8cpp → .cpp
            signature_href = self._cpp_namespace_href_from_signature(page_soup)
            if signature_href:
                candidates.append(signature_href)
            content = page_soup.select_one(".vp-doc") or page_soup.select_one(".content") or page_soup
            if content is not None:
                # Fallback for Exhale pages whose filename does not encode namespace path:
                # infer owner namespace from content links, then from rendered C++ text.
                content_href = self._cpp_namespace_href_from_content_links(content)
                if content_href:
                    candidates.append(content_href)
                text_href = self._cpp_namespace_href_from_cpp_text(content)
                if text_href:
                    candidates.append(text_href)

        for candidate in candidates:
            resolved = self._resolve_cpp_namespace_candidate(candidate)
            if resolved:
                return resolved
        return None

    def _resolve_cpp_namespace_candidate(self, candidate: Optional[str]) -> Optional[str]:
        if not candidate:
            return None
        return self._resolve_known_cpp_namespace_href(candidate)

    def _cpp_namespace_href_from_munged_stem(self, stem: str) -> Optional[str]:
        munged = None
        for prefix in self._CPP_EXHALE_FILE_PREFIXES:
            if stem.startswith(prefix):
                munged = stem[len(prefix):]
                break
        # _1_1 is exhales :: encoding
        if munged is None or "_1_1" not in munged:
            return None
        parts = munged.split("_1_1")
        if len(parts) < 2:
            return None
        return "namespace_" + "__".join(parts[:-1]) + ".html"

    @staticmethod
    def _cpp_namespace_href_from_group_stem(stem: str) -> Optional[str]:
        # Exhale group pages (e.g. function_group__ouster__core__destagger_1g...)
        # often don't encode a full namespace path that matches _1_1 splitting.
        group_match = re.search(r"group__ouster__(?:sdk__)?([a-z0-9_]+)", stem, re.IGNORECASE)
        if group_match:
            group_tokens = [tok for tok in group_match.group(1).split("__") if tok]
            if group_tokens:
                return f"namespace_ouster__sdk__{group_tokens[0]}.html"

        # Some Exhale group pages encode the group as CamelCase, e.g.
        # function_group__OusterCoreTypeGetFormat_1g...
        camel_match = re.search(r"group__ouster([A-Za-z0-9]+)", stem, re.IGNORECASE)
        if not camel_match:
            return None
        namespace_words = re.findall(r"[A-Z]+(?=[A-Z][a-z]|$)|[A-Z]?[a-z]+|\d+", camel_match.group(1))
        if not namespace_words:
            return None
        return f"namespace_ouster__sdk__{namespace_words[0].lower()}.html"

    @staticmethod
    def _cpp_namespace_href_from_content_links(content_root) -> Optional[str]:
        candidates: set[str] = set()
        for link in content_root.select("a.reference.internal[href]"):
            href = (link.get("href") or "").split("#", 1)[0].strip()
            if not href:
                continue
            href_base = href.rsplit("/", 1)[-1]
            if not (href_base.startswith("namespace_") and href_base.endswith(".html")):
                continue
            candidates.add(href_base)

        if not candidates:
            return None
        # Prefer the most specific namespace (largest token depth), and avoid root.
        ranked = sorted(candidates, key=lambda h: (h.count("__"), len(h)), reverse=True)
        for candidate in ranked:
            if candidate != "namespace_ouster__sdk.html":
                return candidate
        return ranked[0]

    def _resolve_known_cpp_namespace_href(self, candidate: str) -> Optional[str]:
        """Resolve candidate namespace to the closest known namespace page."""
        if not candidate.startswith("namespace_") or not candidate.endswith(".html"):
            return None
        cached = self._cpp_namespace_member_groups
        if not cached:
            return candidate
        if candidate in cached:
            return candidate
        tokens = [tok for tok in candidate[len("namespace_"):-len(".html")].split("__") if tok]
        while len(tokens) >= 1:
            maybe = f"namespace_{'__'.join(tokens)}.html"
            if maybe in cached:
                return maybe
            tokens = tokens[:-1]
        return None

    @staticmethod
    def _cpp_namespace_href_from_signature(page_soup) -> Optional[str]:
        """Infer namespace_*.html from C++ signature descclassname text."""
        # Exhale emits:
        # <span class="sig-prename descclassname">ouster::sdk::core::</span>
        # on function/variable/enum detail pages.
        for node in page_soup.select("dt.sig .sig-prename.descclassname"):
            namespace_text = node.get_text(" ", strip=True)
            if not namespace_text:
                continue
            namespace_text = re.sub(r"\s+", "", namespace_text).rstrip(":")
            if "::" not in namespace_text:
                continue
            tokens = [part for part in namespace_text.split("::") if part]
            if not tokens:
                continue
            return f"namespace_{'__'.join(tokens)}.html"
        return None

    @staticmethod
    def _cpp_namespace_href_from_cpp_text(content_root) -> Optional[str]:
        """Infer namespace from rendered cpp text when links/signatures are sparse."""
        text = content_root.get_text(" ", strip=True) if content_root else ""
        if not text:
            return None
        matches = re.findall(r"ouster::sdk::([A-Za-z_][A-Za-z0-9_:]*)", text)
        if not matches:
            return None
        # Pick the most specific path first; caller resolves to known namespace.
        best = max(matches, key=lambda m: (m.count("::"), len(m)))
        tokens = ["ouster", "sdk", *[tok for tok in best.split("::") if tok]]
        return f"namespace_{'__'.join(tokens)}.html"

    def _find_cpp_namespace_li(self, sidebar, html_path: Path, page_soup=None):
        """Return the sidebar <li> for the namespace that owns this C++ API page."""
        ns_href = self._cpp_namespace_href_for_page(html_path, page_soup)
        if not ns_href:
            return None

        def _level(li_tag):
            classes = li_tag.get("class", []) or []
            if isinstance(classes, str):
                classes = classes.split()
            for cls in classes:
                if cls.startswith("toctree-l"):
                    try:
                        return int(cls.replace("toctree-l", ""))
                    except ValueError:
                        continue
            return 0

        best_li = None
        best_level = -1
        for link in sidebar.select("a.reference[href]"):
            href = link.get("href") or ""
            if "#" in href:
                continue
            li = link.find_parent("li")
            if li is None or self._sidebar_li_basename(li) != ns_href:
                continue
            level = _level(li)
            if level >= best_level:
                best_level = level
                best_li = li
        return best_li

    def _remove_existing_api_sections(self, sidebar):
        for existing in sidebar.select("ul.api-section-list"):
            existing.decompose()

    @staticmethod
    def _python_api_content_root(soup):
        return soup.select_one(".vp-doc") or soup.select_one(".content")

    def _find_section_for_heading(self, soup, section_id: str):
        content = self._python_api_content_root(soup)
        if content is None:
            return None
        # IDs like module-ouster.sdk.core.core contain dots; CSS # selectors
        # treat dots as class separators, so use attribute lookup instead.
        section = content.find("section", id=section_id)
        if section is not None:
            return section
        element = content.find(id=section_id)
        if element is None:
            return None
        return element.find_parent("section")

    @staticmethod
    def _is_type_alias_attribute(dl, dt) -> bool:
        """Return True for `dl.py.attribute` or `dl.py.data` entries whose dd says 'alias of'.

        Both ``attribute`` (re-exported names) and ``data`` (TypeAlias / type-alias
        declarations) use the same "alias of …" pattern and should be treated as
        class-like symbols in the sidebar.
        """
        if dl is None or dt is None:
            return False
        dl_classes = dl.get("class", []) or []
        if isinstance(dl_classes, str):
            dl_classes = dl_classes.split()
        if "py" not in dl_classes:
            return False
        if "attribute" not in dl_classes and "data" not in dl_classes:
            return False
        dd = dt.find_next_sibling("dd")
        if dd is None:
            return False
        return "alias of" in dd.get_text(" ", strip=True).casefold()

    def _python_dl_sidebar_kind(self, dl, dt=None) -> str:
        if dl is None:
            return "other"
        classes = dl.get("class", []) or []
        if isinstance(classes, str):
            classes = classes.split()
        if "py" not in classes:
            return "other"
        if "function" in classes:
            return "function"
        if "class" in classes or "exception" in classes:
            return "class"
        if dt is not None and self._is_type_alias_attribute(dl, dt):
            return "class"
        return "other"

    def _prefer_sidebar_anchor(self, soup, symbol_name: str, anchor_id: str) -> tuple[str, str]:
        """Prefer the h2/section slug (e.g. #ousteriotype) over the FQN dt id."""
        short_id = symbol_name.casefold()
        if short_id and soup.find(id=short_id) is not None:
            return f"#{short_id}", short_id
        return f"#{anchor_id}", anchor_id

    def _python_heading_kind(self, soup, section_id: str) -> str:
        section = self._find_section_for_heading(soup, section_id)
        if section is None:
            return "other"
        dl = section.find(
            "dl",
            class_=lambda value: value and any(
                cls.startswith("py") for cls in (value if isinstance(value, list) else [value])
            ),
        )
        if dl is None:
            return "other"
        dt = dl.find("dt", class_=lambda value: value and "sig-object" in (
            value if isinstance(value, list) else [value]
        ))
        return self._python_dl_sidebar_kind(dl, dt)

    @staticmethod
    def _make_api_group_marker_section(soup, section_id: str, title: str):
        section = soup.new_tag("section", id=section_id)
        h2 = soup.new_tag("h2")
        h2.append(NavigableString(title))
        headerlink = soup.new_tag(
            "a",
            href=f"#{section_id}",
            **{"class": "headerlink", "title": "Link to this heading"},
        )
        headerlink.string = "¶"
        h2.append(headerlink)
        section.append(h2)
        return section

    def _ensure_python_group_anchors(
            self,
            soup,
            class_nodes: list,
            function_nodes: list,
    ) -> None:
        content = self._python_api_content_root(soup)
        if content is None:
            return
        has_classes_section = content.select_one("section#classes") is not None
        has_functions_section = content.select_one("section#functions") is not None
        if (not class_nodes or has_classes_section) and (not function_nodes or has_functions_section):
            return

        first_class_section = None
        if class_nodes:
            first_class_id = class_nodes[0].get("id")
            first_class_section = (
                self._find_section_for_heading(soup, first_class_id)
                if first_class_id else None
            )
        first_function_section = None
        if function_nodes:
            first_function_id = function_nodes[0].get("id")
            first_function_section = (
                self._find_section_for_heading(soup, first_function_id)
                if first_function_id else None
            )

        insertion_point = first_class_section or first_function_section
        if first_class_section is not None and first_function_section is not None:
            all_sections = list(content.find_all("section", recursive=False))
            idx_class = (
                all_sections.index(first_class_section)
                if first_class_section in all_sections else 10**9
            )
            idx_function = (
                all_sections.index(first_function_section)
                if first_function_section in all_sections else 10**9
            )
            insertion_point = first_class_section if idx_class <= idx_function else first_function_section
        if insertion_point is not None:
            if class_nodes and not has_classes_section:
                insertion_point.insert_before(
                    self._make_api_group_marker_section(soup, "classes", "Classes"))
            if function_nodes and not has_functions_section:
                insertion_point.insert_before(
                    self._make_api_group_marker_section(soup, "functions", "Functions"))
            return

        # Export-only pages might not have class/function detail sections in the body.
        if class_nodes:
            self._ensure_python_summary_section(soup, "classes", "Classes")
        if function_nodes:
            self._ensure_python_summary_section(soup, "functions", "Functions")

    def _ensure_python_summary_section(
            self, soup, section_id: str, title: str, *, before_section_id: str = "submodules"):
        content = self._python_api_content_root(soup)
        if content is None:
            return None
        existing = content.select_one(f"section#{section_id}")
        if existing is not None:
            return existing
        new_section = self._make_api_group_marker_section(soup, section_id, title)
        before_section = content.select_one(f"section#{before_section_id}")
        if before_section is not None:
            before_section.insert_before(new_section)
        else:
            content.append(new_section)
        return new_section

    @staticmethod
    def _append_api_section_link(parent, soup, node: dict, *, label: str | None = None) -> None:
        href = node.get("href")
        if not href:
            node_id = node.get("id")
            if not node_id:
                return
            href = f"#{node_id}"
        link = soup.new_tag("a", href=href)
        if not href.startswith("#"):
            link["class"] = ["reference", "internal"]
            link["data-cpp-nav-clean"] = "1"
        text = label if label is not None else node.get("label", node.get("title", ""))
        if not text:
            return
        link.string = text
        item = soup.new_tag("li")
        item.append(link)
        parent.append(item)

    def _append_api_section_group(
            self, sections_ul, soup, anchor_id: str, label: str, members: list) -> None:
        if not members:
            return
        group_li = soup.new_tag("li")
        group_link = soup.new_tag("a", href=f"#{anchor_id}")
        group_link.string = label
        group_li.append(group_link)
        sub_ul = soup.new_tag("ul")
        for node in members:
            self._append_api_section_link(
                sub_ul, soup, node, label=node.get("label"))
        group_li.append(sub_ul)
        sections_ul.append(group_li)

    def _resolve_symbol_anchor(
            self, soup, page_module: str, submodule: str | None, symbol_name: str) -> Optional[str]:
        package_id = f"{page_module}.{symbol_name}"
        if soup.find(id=package_id) is not None:
            return package_id
        if submodule:
            nested_id = f"{page_module}.{submodule}.{symbol_name}"
            if soup.find(id=nested_id) is not None:
                return nested_id
        return None

    def _export_member_href(
            self, soup, page_module: str, export: PublicExport) -> Optional[str]:
        anchor = self._resolve_symbol_anchor(
            soup, page_module, export.source_submodule, export.name)
        if anchor:
            return f"#{anchor}"
        if export.source_submodule:
            nested_id = f"{page_module}.{export.source_submodule}.{export.name}"
            return f"{page_module}.{export.source_submodule}.html#{nested_id}"
        return None

    def _python_symbol_kind(self, soup, symbol_id: str) -> str:
        dt = soup.find(id=symbol_id)
        if dt is None:
            return "other"
        dl = dt.find_parent("dl")
        return self._python_dl_sidebar_kind(dl, dt)

    def _sidebar_entry_from_export(
            self, soup, page_module: str, export: PublicExport) -> Optional[dict]:
        href = self._export_member_href(soup, page_module, export)
        if not href:
            return None
        if href.startswith("#"):
            anchor = href.lstrip("#")
            kind = self._python_symbol_kind(soup, anchor)
            href, anchor = self._prefer_sidebar_anchor(soup, export.name, anchor)
            return {
                "id": anchor,
                "href": href,
                "label": export.name,
                "title": export.name,
                "kind": kind,
            }
        return {
            "href": href,
            "label": export.name,
            "title": export.name,
            "kind": self._runtime_symbol_kind(page_module, export.name),
        }

    def _runtime_symbol_kind(self, page_module: str, symbol_name: str) -> str:
        kind_map = self._python_symbol_kinds_cache.get(page_module)
        if kind_map is None:
            kind_map = {}
            try:
                module_obj = importlib.import_module(page_module)
                for attr_name in dir(module_obj):
                    if attr_name.startswith("_"):
                        continue
                    try:
                        value = getattr(module_obj, attr_name)
                    except Exception:
                        continue
                    if inspect.isclass(value):
                        kind_map[attr_name] = "class"
                    elif callable(value):
                        kind_map[attr_name] = "function"
            except Exception:
                kind_map = {}
            self._python_symbol_kinds_cache[page_module] = kind_map
        return kind_map.get(symbol_name, "other")

    @staticmethod
    def _sidebar_node_label_key(node: dict) -> str:
        return node.get("label", node["title"]).casefold()

    @staticmethod
    def _sidebar_node_anchor_id(node: dict) -> str:
        anchor_id = (node.get("id") or "").strip()
        if anchor_id:
            return anchor_id
        href = (node.get("href") or "").strip()
        if href.startswith("#"):
            return href.lstrip("#")
        return ""

    def _append_python_submodules_group(
            self, sections_ul, soup, submodules: list) -> None:
        self._append_api_section_group(
            sections_ul, soup, "submodules", "Submodules", submodules)

    @staticmethod
    def _normalize_submodule_label(label: str, page_module: str) -> str:
        cleaned = re.sub(r"\s+module$", "", (label or "").strip())
        if not cleaned:
            return ""
        if cleaned.startswith("ouster.sdk."):
            # str.removeprefix is 3.9+; docs runtime uses ≥3.9, mypy target is 3.8
            cleaned = cleaned.removeprefix("ouster.sdk.")  # type: ignore[attr-defined]
        module_short = page_module.removeprefix("ouster.sdk.")  # type: ignore[attr-defined]
        if module_short and cleaned.startswith(module_short + "."):
            cleaned = cleaned[len(module_short) + 1:]
        if "." in cleaned:
            cleaned = cleaned.rsplit(".", 1)[-1]
        return cleaned.strip()

    @classmethod
    def _filter_python_submodule_entries(
            cls, entries: list[dict], page_module: str, exported_shorts: set[str]) -> list[dict]:
        filtered: list[dict] = []
        seen_labels: set[str] = set()
        for entry in entries:
            normalized = cls._normalize_submodule_label(
                entry.get("label", ""), page_module
            )
            if not normalized or normalized in seen_labels:
                continue
            if page_module != "ouster.sdk" and normalized in exported_shorts:
                continue
            seen_labels.add(normalized)
            filtered.append({
                **entry,
                "label": normalized,
                "title": normalized,
            })
        return filtered

    @classmethod
    def _extract_submodule_nodes_from_toctree(
            cls, soup, section_id: str) -> list[dict]:
        section = soup.find(id=section_id)
        if section is None:
            return []
        if section.name != "section":
            section = section.find_parent("section")
        if section is None:
            return []
        nodes: list[dict] = []
        for link in section.select(".toctree-wrapper a.reference.internal[href]"):
            href = (link.get("href") or "").strip()
            if not href or href.startswith("#"):
                continue
            label = link.get_text(" ", strip=True)
            if not label:
                continue
            nodes.append({
                "href": href,
                "label": label,
                "title": label,
            })
        return nodes

    @staticmethod
    def _format_python_signature_text(raw: str) -> str:
        text = re.sub(r"\s+", " ", raw.strip())
        text = re.sub(r"\s+\(", "(", text)
        text = re.sub(r"\(\s+", "(", text)
        text = re.sub(r"\s+\)", ")", text)
        text = re.sub(r"\s*,\s*", ", ", text)
        text = re.sub(r"\s*->\s*", " -> ", text)
        return text

    @classmethod
    def _find_signature_dt(cls, soup, anchor_id: str):
        if not anchor_id:
            return None
        dt = soup.find("dt", id=anchor_id)
        if dt is not None and cls._is_top_level_signature_dt(dt):
            return dt
        section = soup.find(id=anchor_id)
        if section is None:
            return dt if dt is not None and "sig-object" in (dt.get("class") or []) else None
        if section.name != "section":
            section = section.find_parent("section")
        if section is None:
            return dt
        for candidate in section.find_all("dt", class_=lambda value: value and "sig-object" in value):
            if not cls._is_top_level_signature_dt(candidate):
                continue
            return candidate
        return dt if dt is not None and "sig-object" in (dt.get("class") or []) else None

    @staticmethod
    def _is_top_level_signature_dt(dt) -> bool:
        dl = dt.find_parent("dl")
        if dl is None:
            return False
        dl_classes = dl.get("class", []) or []
        if isinstance(dl_classes, str):
            dl_classes = dl_classes.split()
        if any(kind in dl_classes for kind in ("method", "attribute", "property")):
            return False
        if "class" in dl_classes or "exception" in dl_classes or "function" in dl_classes:
            return True
        em = dt.find("em", class_="property")
        if em is not None:
            kind = em.get_text(" ", strip=True).casefold()
            return kind in {"class", "exception", "function", "async function"}
        return "sig-object" in (dt.get("class") or [])

    def _extract_python_symbol_signature(self, soup, anchor_id: str) -> Optional[str]:
        dt = self._find_signature_dt(soup, anchor_id)
        if dt is None:
            return None
        fragment = BeautifulSoup(str(dt), "html.parser")
        work = fragment.find("dt")
        if not isinstance(work, Tag):
            return None
        for tag in work.select("a.headerlink, a.reference.external, .viewcode-link"):
            tag.decompose()
        return self._format_python_signature_text(work.get_text(" ", strip=True))

    @staticmethod
    def _summary_anchor_id(entry: dict, target_href: str) -> Optional[str]:
        entry_id = entry.get("id")
        if entry_id:
            return entry_id
        if target_href.startswith("#"):
            return target_href[1:] or None
        fragment = posixpath.basename(target_href).split("#", 1)
        if len(fragment) == 2 and fragment[1]:
            return fragment[1]
        return None

    @staticmethod
    def _summary_signature_fallback(section_id: str, label: str) -> Optional[str]:
        if section_id == "classes":
            return f"class {label}"
        if section_id == "functions":
            return f"def {label}(...)"
        if section_id == "submodules":
            return f"module {label}"
        return None

    def _append_summary_signature_link(
            self, soup, p, target_href: str, label: str, signature: str) -> None:
        link = soup.new_tag("a", href=target_href)
        link["class"] = ["reference", "internal"]
        code = soup.new_tag("code")
        span = soup.new_tag("span", attrs={"class": "pre"})
        span.string = signature
        code.append(span)
        link.append(code)
        p.append(link)

    def _replace_section_summary_list(self, soup, section_id: str, entries: list) -> None:
        section = soup.find(id=section_id)
        if section is None:
            return
        if section.name != "section":
            section = section.find_parent("section")
        if section is None:
            return

        for child in list(section.children):
            name = getattr(child, "name", None)
            if name in {"ul", "ol"}:
                child.decompose()
                continue
            if name == "div" and "toctree-wrapper" in (child.get("class") or []):
                child.decompose()

        if not entries:
            return

        summary_ul = soup.new_tag("ul", attrs={"class": "simple"})
        for entry in entries:
            target_href = entry.get("href")
            entry_id = entry.get("id")
            if not target_href:
                if entry_id:
                    target_href = f"#{entry_id}"
            if not target_href:
                continue
            label = entry.get("label", entry.get("title", "")).strip()
            if not label:
                continue
            anchor_id = self._summary_anchor_id(entry, target_href)
            if target_href.startswith("#") and anchor_id and soup.find(id=anchor_id) is None:
                continue
            is_submodule_entry = section_id == "submodules" or (
                anchor_id and anchor_id.startswith("module-")
            )
            signature = None
            if not is_submodule_entry and not (".html" in target_href and not target_href.startswith("#")):
                if anchor_id:
                    signature = self._extract_python_symbol_signature(soup, anchor_id)
            if not is_submodule_entry and not signature:
                signature = self._summary_signature_fallback(section_id, label)
            li = soup.new_tag("li")
            p = soup.new_tag("p")
            if signature:
                self._append_summary_signature_link(soup, p, target_href, label, signature)
            else:
                link = soup.new_tag("a", href=target_href)
                link["class"] = ["reference", "internal"]
                link.string = label
                p.append(link)
            li.append(p)
            summary_ul.append(li)
        if summary_ul.contents:
            section.append(summary_ul)

    def _populate_python_api_section_list(
            self, soup, sections_ul, heading_nodes: list, page_module: str) -> None:
        layout = get_package_analysis(self.app, page_module)
        pinned: list = []
        class_by_name: dict[str, dict] = {}
        function_by_name: dict[str, dict] = {}
        submodules_only: list = []
        submodules_header_node: Optional[dict] = None

        for node in heading_nodes:
            if node["level"] != 2:
                continue
            node_id = node.get("id")
            if not node_id:
                continue
            title_key = node["title"].casefold()
            if title_key == "submodules":
                submodules_header_node = node
            if title_key in self._PYTHON_API_GROUP_TITLES:
                continue
            if title_key in self._PYTHON_API_PINNED_TITLES:
                pinned.append(node)
                continue
            if node_id.startswith("module-"):
                fqn = node_id.removeprefix("module-")  # type: ignore[attr-defined]
                if fqn == page_module:
                    pinned.append(node)
                    continue
                short = fqn.removeprefix(page_module + ".")  # type: ignore[attr-defined]
                short = self._normalize_submodule_label(short, page_module)
                if short:
                    submodules_only.append({**node, "label": short})
                continue
            kind = self._python_heading_kind(soup, node_id)
            if kind == "class":
                title = node.get("title", "")
                if title:
                    href, anchor = self._prefer_sidebar_anchor(soup, title, node_id)
                    node = {**node, "id": anchor, "href": href}
                class_by_name.setdefault(self._sidebar_node_label_key(node), node)
            elif kind == "function":
                function_by_name.setdefault(self._sidebar_node_label_key(node), node)

        for export in layout.public_exports:
            key = export.name.casefold()
            if key in class_by_name or key in function_by_name:
                continue
            entry = self._sidebar_entry_from_export(soup, page_module, export)
            if entry is None:
                continue
            entry_kind: Optional[str] = entry.get("kind")
            if entry_kind == "function":
                function_by_name.setdefault(key, entry)
            elif entry_kind == "class":
                class_by_name.setdefault(key, entry)

        classes = sorted(class_by_name.values(), key=self._sidebar_node_label_key)
        functions = sorted(function_by_name.values(), key=self._sidebar_node_label_key)
        submodules_only = self._filter_python_submodule_entries(
            submodules_only, page_module, set(layout.exported_submodule_shorts)
        )
        submodules_source = "headings"
        if not submodules_only:
            toctree_nodes = self._extract_submodule_nodes_from_toctree(
                soup, "submodules")
            submodules_only = self._filter_python_submodule_entries(
                toctree_nodes, page_module, set(layout.exported_submodule_shorts)
            )
            submodules_source = "toctree"
        self.logger.info(
            "%s: submodules source=%s labels=%s exported=%s",
            page_module,
            submodules_source,
            [node.get("label") for node in submodules_only],
            sorted(layout.exported_submodule_shorts),
        )
        submodules_only.sort(key=self._sidebar_node_label_key)
        if submodules_header_node is not None and not submodules_only:
            pinned.append(submodules_header_node)

        self._ensure_python_group_anchors(soup, classes, functions)
        self._replace_section_summary_list(soup, "classes", classes)
        self._replace_section_summary_list(soup, "functions", functions)
        self._replace_section_summary_list(soup, "submodules", submodules_only)

        for node in pinned:
            self._append_api_section_link(sections_ul, soup, node)
        self._append_api_section_group(sections_ul, soup, "classes", "Classes", classes)
        self._append_api_section_group(sections_ul, soup, "functions", "Functions", functions)
        self._append_python_submodules_group(sections_ul, soup, submodules_only)

    def _section_for_sidebar_node(self, soup, node: dict):
        anchor_id = self._sidebar_node_anchor_id(node)
        if not anchor_id:
            return None
        return self._find_section_for_heading(soup, anchor_id)

    def _extract_heading_nodes_for_api(self, soup):
        content_root = soup.select_one(".vp-doc") or soup.select_one(".content")
        if content_root is None:
            return []
        nodes = []
        for header in content_root.find_all(["h2", "h3"]):
            hid = header.get("id")
            if not hid:
                parent_section = header.find_parent("section")
                if parent_section:
                    hid = parent_section.get("id")
            if not hid:
                header_link = header.find("a", class_="headerlink")
                if header_link:
                    href = header_link.get("href")
                    if href and href.startswith("#"):
                        hid = href.lstrip("#")
            if not hid:
                continue
            classes = header.get("class", [])
            if "no-right-toc" in classes:
                continue
            level = int(header.name[1])
            if level < 2 or level > 3:
                continue
            title = header.get_text(" ", strip=True)
            header_link = header.find("a", class_="headerlink")
            if header_link:
                marker = header_link.get_text(" ", strip=True)
                if marker:
                    title = title.replace(marker, "").strip()
            if not title:
                continue
            nodes.append({"level": level, "id": hid, "title": title})
        return nodes

    def _normalize_python_module_labels(self, sidebar, rel_path):
        prefix = "ouster.sdk."
        for link in sidebar.find_all("a"):
            href = link.get("href") or ""
            if href.startswith("#module-"):
                continue
            text = link.get_text(strip=True)
            if not text or text.strip() == "ouster.sdk" or prefix not in text:
                continue
            new_text = text.replace(prefix, "")
            self._safe_replace_link_text(link, new_text, f"{rel_path} python module '{text}'")

    def _normalize_cpp_namespace_labels(self, sidebar, rel_path):
        prefix = "ouster::sdk::"
        for link in sidebar.find_all("a"):
            text = link.get_text(strip=True)
            if not text:
                continue
            if text.startswith("Namespace "):
                self._safe_replace_link_text(
                    link, self._clean_cpp_sidebar_label(text),
                    f"{rel_path} cpp namespace '{text}'")
                text = link.get_text(strip=True)
            if text.strip() == "ouster" or prefix not in text:
                continue
            new_text = text.replace(prefix, "")
            self._safe_replace_link_text(link, new_text, f"{rel_path} cpp namespace '{text}'")

    def _safe_replace_link_text(self, link, new_text, context=""):
        if not isinstance(new_text, str):
            return
        new_text = new_text.strip()
        if not new_text:
            return
        try:
            link.clear()
            link.string = new_text
        except Exception as exc:
            self.logger.warning(f"failed to replace text for {context}: {exc}")
            self.logger.debug(f"link repr: {link!r}")
            self.logger.debug(traceback.format_exc())
            raise

    def _clone_tag(self, tag):
        # Avoid re-parsing HTML to limit BeautifulSoup nesting churn.
        return copy.copy(tag)

    def _prepare_cpp_namespace_root(self, root_li):
        if root_li is None:
            return None

        root_anchor = root_li.find("a", href="namespace_ouster__sdk.html")
        for anchor in list(root_li.select('a[href="namespace_ouster__sdk.html"]')):
            if root_anchor and anchor is root_anchor:
                continue
            li_to_remove = anchor.find_parent("li")
            if li_to_remove:
                li_to_remove.decompose()

        page_deprecated_href = "page_deprecated.html"
        has_deprecated = any(
            link.get("href") == page_deprecated_href for link in root_li.select("a.reference")
        )
        if not has_deprecated:
            factory = BeautifulSoup("", "html.parser")
            nav_ul = root_li.find("ul")
            if nav_ul is None:
                nav_ul = factory.new_tag("ul")
                root_li.append(nav_ul)
            new_li = factory.new_tag("li", **{"class": "toctree-l2"})
            new_div = factory.new_tag("div", **{"class": "sidebar-link-row"})
            new_link = factory.new_tag("a", href=page_deprecated_href, **{"class": "reference internal"})
            new_link.string = "Deprecated APIs"
            new_div.append(new_link)
            new_li.append(new_div)
            nav_ul.append(new_li)

        for li in root_li.find_all("li"):
            classes = [cls for cls in (li.get("class", []) or []) if cls != "current"]
            if classes:
                li["class"] = classes
            else:
                li.attrs.pop("class", None)
        for ul in root_li.find_all("ul"):
            ul_classes = [cls for cls in (ul.get("class", []) or []) if cls != "current"]
            if ul_classes:
                ul["class"] = ul_classes
            else:
                ul.attrs.pop("class", None)
        root_li["class"] = ["toctree-l1"]
        return root_li

    @staticmethod
    def _set_nav_ul_open(nav_ul, is_open: bool) -> None:
        if nav_ul is None:
            return
        child_classes = [
            cls for cls in (nav_ul.get("class", []) or []) if cls != "collapsed"
        ]
        if not is_open:
            child_classes.append("collapsed")
        if child_classes:
            nav_ul["class"] = child_classes
        else:
            nav_ul.attrs.pop("class", None)

    def _ensure_sidebar_toggle(self, li, is_open, nav_ul=None):
        if li is None:
            return
        factory = BeautifulSoup("", "html.parser")
        if nav_ul is None:
            api_section_ul = None
            for child_ul in li.find_all("ul", recursive=False):
                classes = child_ul.get("class", []) or []
                if isinstance(classes, str):
                    classes = classes.split()
                if "api-section-list" in classes:
                    api_section_ul = child_ul
                    continue
                nav_ul = child_ul
                break
            if nav_ul is None:
                nav_ul = api_section_ul
        if nav_ul is None or not nav_ul.contents:
            return

        link = li.find("a", class_="reference")
        if link is None:
            return

        link_row = link.find_parent("div", class_="sidebar-link-row")
        if link_row is None:
            link_row = factory.new_tag("div", **{"class": "sidebar-link-row"})
            link.replace_with(link_row)
            link_row.append(link)

        toggle = link_row.find("button", class_="sidebar-toggle")
        if toggle is None:
            toggle = factory.new_tag("button", type="button", **{"class": "sidebar-toggle"})
            hidden = factory.new_tag("span", **{"class": "visually-hidden"})
            hidden.string = "Toggle section"
            toggle.append(hidden)
            arrow = BeautifulSoup(self._SIDEBAR_TOGGLE_ARROW_SVG, "html.parser").find("svg")
            if arrow:
                toggle.append(arrow)
            link_row.append(toggle)

        link_text = link.get_text(strip=True) or "section"
        toggle["aria-label"] = f"Toggle section {link_text}"
        toggle_classes = ["sidebar-toggle"]
        if is_open:
            toggle_classes.append("is-open")
        toggle["class"] = toggle_classes
        toggle["aria-expanded"] = "true" if is_open else "false"

        self._set_nav_ul_open(nav_ul, is_open)

    def _apply_sidebar_toggles(self, root_li):
        if root_li is None:
            return
        self._ensure_sidebar_toggle(root_li, is_open=True)
        for li in root_li.find_all("li"):
            if li is root_li:
                continue
            self._ensure_sidebar_toggle(li, is_open=False)

    def _load_namespace_root_structure(self) -> Optional[BeautifulSoup]:
        """Return a cloned <li> tree rooted at ouster::sdk."""
        if self._cpp_namespace_root is None:
            return None
        root_li = self._clone_tag(self._cpp_namespace_root)
        if root_li is None:
            return None
        return root_li

    def _adjust_cpp_namespace_tree(self, sidebar, rel_path, page_soup):
        normalized_rel = rel_path.replace("\\", "/")

        needle = f"/{self.exhale_cpp_dir}/{self.exhale_api_dir}/"

        if needle not in normalized_rel and not normalized_rel.startswith(
            f"{self.exhale_cpp_dir}/{self.exhale_api_dir}/"
        ):
            return

        existing_ul = sidebar.select_one("ul[data-cpp-nav-injected='1']")
        if existing_ul:
            return

        cached_root = self._load_namespace_root_structure()
        if cached_root is None:
            return

        self._apply_sidebar_toggles(cached_root)

        for link in list(cached_root.select("a.reference")):
            text = link.get_text(strip=True)
            if text.startswith("Namespace "):
                cleaned = text.replace("Namespace ", "", 1)
                self._safe_replace_link_text(link, cleaned, f"{rel_path} cpp nav child")
            link_classes = [cls for cls in (link.get("class", []) or []) if cls != "current"]
            if link_classes:
                link["class"] = link_classes
            else:
                link.attrs.pop("class", None)
            link["data-cpp-nav-clean"] = "1"

        cached_root["class"] = ["toctree-l1", "current"]
        root_anchor = cached_root.find("a", class_="reference")
        if root_anchor is not None:
            root_anchor["href"] = "namespace_ouster__sdk.html"
            root_anchor["data-cpp-nav-clean"] = "1"
            classes = set(root_anchor.get("class", []) or [])
            classes.update(["reference", "internal", "current"])
            root_anchor["class"] = list(classes)
        else:
            self.logger.debug(f"{normalized_rel}: cached root missing anchor, skipping link update")

        group_container = sidebar.select_one(".sidebar-group")
        if group_container is None:
            group_container = sidebar.new_tag("div", **{"class": "sidebar-group"})
            sidebar.append(group_container)
        else:
            self.logger.debug(f"{normalized_rel}: found existing sidebar-group container")

        group_ul = group_container.find("ul")
        if group_ul is None:
            group_ul = sidebar.new_tag("ul")
            group_container.append(group_ul)
        elif group_ul.get("data-cpp-nav-injected") == "1":
            self.logger.debug(f"{normalized_rel}: sidebar already injected, skipping rewrite")
            return
        else:
            self.logger.debug(f"{normalized_rel}: clearing sidebar-group ul contents")
        group_ul.clear()
        group_ul.attrs.pop("class", None)
        group_ul.append(cached_root)
        group_ul["data-cpp-nav-injected"] = "1"


def setup(app: Sphinx):
    manager = PostBuildManager(app)
    manager.register()
    app.ouster_post_build_manager = manager
    return {"parallel_read_safe": True, "parallel_write_safe": True}
