"""
Rewrite Exhale-generated function RST to render group-only functions via doxygengroup.
"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

from sphinx.application import Sphinx
from sphinx.util import logging


logger = logging.getLogger(__name__)

_FUNC_FILE_RE = re.compile(r"^function_(?P<refid>.+)\.rst$")
_FUNC_TITLE_RE = re.compile(r"^Function\s+(?P<name>.+)$")
_DOXYGENFUNCTION_RE = re.compile(r"^\.\.\s+doxygenfunction::\s+(?P<name>.+)\(\s*\)\s*$")


def _compound_from_refid(refid: str) -> str:
    """Strip Doxygen's hash suffix to get the compound refid."""
    if "_1" in refid:
        return refid.split("_1", 1)[0]
    return refid


def _load_group_name(xml_dir: Path, refid: str) -> str | None:
    """Resolve a group refid to its compound name from XML."""
    if not refid.startswith("group__"):
        return None
    group_refid = _compound_from_refid(refid)
    xml_path = xml_dir / f"{group_refid}.xml"
    if not xml_path.exists():
        logger.info(
            "Exhale overload fix: group XML not found for refid=%s at %s",
            refid,
            xml_path,
        )
        return None
    try:
        tree = ET.parse(xml_path)
    except Exception:
        logger.info(
            "Exhale overload fix: failed to parse group XML for refid=%s at %s",
            refid,
            xml_path,
        )
        return None
    root = tree.getroot()
    compoundname = root.findtext(".//compoundname")
    if compoundname:
        return compoundname.strip()
    return None


def _normalize_group_directive(lines: list[str], idx: int) -> bool:
    """Ensure a doxygengroup directive has a stable set of options."""
    changed = False
    insert_idx = idx + 1

    # Detect indentation from first option line, default to 3 spaces
    indent = "   "
    option_match = re.match(r'(\s+):', lines[insert_idx]) if insert_idx < len(lines) else None
    if option_match:
        detected_indent = option_match.group(1)
        indent = detected_indent

    # Remove duplicate project/content-only options if present.
    option_pattern = re.compile(r'\s+:')
    removed_options = []
    while insert_idx < len(lines):
        line = lines[insert_idx]
        if not option_pattern.match(line):
            break
        opt = line.strip()
        if opt in {":project: cpp_api", ":content-only:"}:
            removed_options.append(opt)
            lines.pop(insert_idx)
            changed = True
            continue
        insert_idx += 1

    # Ensure members/project options exist once.
    needed = [":members:", ":project: cpp_api"]
    existing = {
        line.strip() for line in lines[idx + 1:idx + 4]
        if option_pattern.match(line)
    }

    add = [opt for opt in needed if opt not in existing]
    if add:
        logger.info("Adding missing options with indentation %r: %s", indent, add)
        lines[idx + 1:idx + 1] = [f"{indent}{opt}" for opt in add]
        changed = True
    return changed


def _rewrite_function_rst(rst_path: Path, group_name: str) -> bool:
    """Rewrite a group function page to use doxygengroup."""
    text = rst_path.read_text(encoding="utf-8")
    lines = text.splitlines()
    changed = False

    title_done = False
    saw_anchor = False
    for idx, line in enumerate(lines):
        if line.startswith(".. _exhale_function_"):
            saw_anchor = True
        m = _FUNC_TITLE_RE.match(line)
        if m and not title_done and saw_anchor:
            new_title = f"Group {group_name}"
            if line != new_title:
                lines[idx] = new_title
                if idx + 1 < len(lines) and set(lines[idx + 1]) == {"="}:
                    lines[idx + 1] = "=" * len(new_title)
                changed = True
            title_done = True
        m = _DOXYGENFUNCTION_RE.match(line)
        if m:
            new_line = f".. doxygengroup:: {group_name}"
            if line != new_line:
                lines[idx] = new_line
                changed = True
            if _normalize_group_directive(lines, idx):
                changed = True
            continue
        if line.strip().startswith(".. doxygenfunction::"):
            # Skip files that already have an explicit signature.
            return False

    if changed:
        rst_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        logger.info("Rewrote overload signature in %s", rst_path)
    else:
        logger.info("Exhale overload fix: no changes needed in %s", rst_path)
    return changed


def _fix_exhale_overloads(app: Sphinx, _env, _docnames) -> None:
    xml_dir = Path(app.doctreedir) / "xml"
    if not xml_dir.exists():
        logger.info("Exhale overload fix skipped: XML dir not found at %s", xml_dir)
        return

    args = getattr(app.config, "exhale_args", {}) or {}
    folder = args.get("containmentFolder", "./cpp/api_cpp").strip("./")
    rst_dir = Path(app.confdir) / folder
    if not rst_dir.exists():
        logger.info("Exhale overload fix skipped: RST dir not found at %s", rst_dir)
        return

    rst_files = list(rst_dir.glob("function_group__*.rst"))
    logger.info("Exhale overload fix: scanning %d RST files in %s", len(rst_files), rst_dir)
    updated = 0
    for rst_path in rst_files:
        m = _FUNC_FILE_RE.match(rst_path.name)
        if not m:
            continue
        refid = m.group("refid")
        group_name = _load_group_name(xml_dir, refid)
        if not group_name:
            logger.info("Exhale overload fix: no group for refid=%s", refid)
            continue
        if _rewrite_function_rst(rst_path, group_name):
            updated += 1

    logger.info("Exhale overload fix updated %d RST files", updated)


def _enable_breathe_group_functions(app: Sphinx) -> None:
    """Allow doxygenfunction to match functions defined only in group XML."""
    try:
        from breathe.renderer.filter import FilterFactory
    except Exception as exc:
        logger.info("Exhale overload fix: unable to import Breathe filters: %s", exc)
        return

    def _create_function_and_all_friend_finder_filter(self, namespace: str, name: str):
        # Use namespace-agnostic filters so group functions are included.
        function_filter = self.create_member_finder_filter("", name, "function")
        friend_filter = self.create_member_finder_filter("", name, "friend")
        return function_filter | friend_filter

    FilterFactory.create_function_and_all_friend_finder_filter = _create_function_and_all_friend_finder_filter


def setup(app: Sphinx):
    _enable_breathe_group_functions(app)
    app.connect("env-before-read-docs", _fix_exhale_overloads)
    return {"version": "1.0", "parallel_read_safe": True}
