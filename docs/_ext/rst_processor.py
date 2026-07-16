"""
RST processor: single pass over generated ouster.sdk*.rst files.
  - Adds pybind sections
"""
from __future__ import annotations

import ast
import re
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    from tree_sitter import Node, Parser
from util import configure_logger
from nanobind import discover_nb, get_stub_param_types
from python_module_ast import get_package_analysis

deprecation_last_version_default = "1.0"

_CPP_PARSER_DEPS_MSG = (
    "tree-sitter and tree-sitter-cpp are required to extract deprecations from "
    "C++ binding sources (python/src/cpp). Install with:\n"
    "  pip install tree-sitter tree-sitter-cpp\n"
    "(also listed in sdk-extensions/docs/requirements.txt)"
)


class RstProcessor:
    _CPP_CLASS_RE = re.compile(r'py::class_<[^>]+>\s*\(\s*[^,]+,\s*"([^"]+)"')

    def __init__(self, app):
        self.app = app
        self.log = configure_logger("ouster.docs.rstproc")
        self._cpp_parser: Optional[Any] = None

    def process_all(self):
        api_dir = Path(self.app.confdir) / "python" / "api_python"
        rst_files = list(api_dir.glob("ouster.sdk*.rst"))
        self.log.info(f"Processing {len(rst_files)} RST files in single pass")

        stats = {"modified": 0, "skipped": 0, "pybind_added": 0, "deprecated_entries": 0}

        for rst_file in rst_files:
            original = rst_file.read_text(encoding="utf-8")
            module_name = rst_file.stem
            self.log.info(
                "Processing %s (%d lines, automodule=%d autoclass=%d)",
                module_name,
                original.count("\n") + (1 if original else 0),
                original.count(".. automodule::"),
                original.count(".. autoclass::"),
            )
            modified = self._transform_rst_content(original, module_name, stats)

            if modified != original:
                rst_file.write_text(modified, encoding="utf-8")
                stats["modified"] += 1
                self.log.info(
                    "Updated %s (%d -> %d lines, automodule=%d autoclass=%d)",
                    module_name,
                    original.count("\n") + (1 if original else 0),
                    modified.count("\n") + (1 if modified else 0),
                    modified.count(".. automodule::"),
                    modified.count(".. autoclass::"),
                )
            else:
                stats["skipped"] += 1
                self.log.info("No changes for %s", module_name)

        stats["deprecated_entries"] = self._write_deprecations_page(api_dir)
        self.log.info(f"RST processing complete: {stats}")

    _SHOW_INHERITANCE_NEXT_TITLE_RE = re.compile(
        r"(:show-inheritance:)\n(?=[A-Za-z_])",
    )

    def _transform_rst_content(self, content: str, module_name: str, stats: Dict) -> str:
        content = self._ensure_rst_section_separators(content)
        if module_name == "ouster.sdk":
            new_content = self._reorder_sdk_subpackages(content)
            if new_content != content:
                content = new_content
                stats.setdefault("reordered", 0)
                stats["reordered"] += 1
                self.log.debug("Reordered ouster.sdk subpackages toctree")
            new_content = self._add_sdk_deprecated_page_entry(content)
            if new_content != content:
                content = new_content
                stats.setdefault("deprecated_page_linked", 0)
                stats["deprecated_page_linked"] += 1
                self.log.debug("Added page_deprecated entry to ouster.sdk toctree")
        # Pybind sections
        if (module_name != "ouster.sdk"
                and not module_name.endswith("._utils")):
            new_content = self._add_pybind_sections(content, module_name)
            if new_content != content:
                content = new_content
                stats["pybind_added"] += 1
                self.log.debug(f"Added pybind sections to {module_name}")
            new_content = self._merge_sort_api_sections(content, module_name)
            if new_content != content:
                content = new_content
                stats.setdefault("sections_merged", 0)
                stats["sections_merged"] += 1
                self.log.debug(f"Merged-sorted API sections for {module_name}")
        return content

    @classmethod
    def _ensure_rst_section_separators(cls, content: str) -> str:
        """Ensure a blank line after automodule options before the next section title."""
        return cls._SHOW_INHERITANCE_NEXT_TITLE_RE.sub(r"\1\n\n", content)

    # ---------------- Pybind Sections ----------------
    def _add_pybind_sections(self, content: str, module_name: str) -> str:
        data = discover_nb(self.app, module_name)
        if not data["classes"] and not data["functions"]:
            self.log.warning(
                "%s: no pybind sections added (discover_nb returned empty)",
                module_name,
            )
            return content

        self.log.info(
            "%s: adding %d autoclass and %d autofunction sections",
            module_name,
            len(data["classes"]),
            len(data["functions"]),
        )

        def _sort_key(item: Dict[str, str]) -> str:
            fq = item.get("name", "")
            prefix = f"{module_name}."
            short = fq[len(prefix):] if fq.startswith(prefix) else fq
            return short.strip().lower()

        lines: List[str] = []
        if data["classes"]:
            for cls in sorted(data["classes"], key=_sort_key):
                fq = cls["name"]
                fq_heading = fq.split(f"{module_name}.")[1]
                lines.extend([
                    f"{fq_heading}",
                    "-" * len(fq_heading),
                    ""
                ])
                lines.append(f".. autoclass:: {fq}")
                lines.append("   :members:")
                lines.append("   :undoc-members:")
                lines.append("   :special-members:")
                lines.append("")
        if data["functions"]:
            for fn in sorted(data["functions"], key=_sort_key):
                fq = fn["name"]
                fq_heading = fq.split(f"{module_name}.")[1]
                lines.extend([
                    f"{fq_heading}",
                    "-" * len(fq_heading),
                    ""
                ])
                lines.append(f".. autofunction:: {fq}")
                lines.append("")

        block = "\n".join(lines)
        return content.rstrip() + "\n\n" + block

    def _merge_sort_api_sections(self, content: str, module_name: str) -> str:
        if module_name == "ouster.sdk":
            return content

        layout = get_package_analysis(self.app, module_name)

        # "Submodules" heading is placed after classes/functions so the page
        # body reads: Module contents → classes → functions → Submodules → submodule detail.
        # "Module contents" and "Subpackages" stay pinned at the top.
        top_pinned_titles = frozenset({"Module contents", "Subpackages"})
        pinned_h2_titles = frozenset({"Module contents", "Subpackages", "Submodules"})

        preamble, parts = self._split_rst_sections(content, module_name)
        if not parts:
            return content

        pinned: List[str] = []
        submodules_heading: List[str] = []
        class_sections: List[str] = []
        apidoc_only_submodules: List[str] = []
        functions: List[str] = []

        fqn_prefix = module_name + "."
        for part in parts:
            title = part.split("\n", 1)[0].strip()
            if title in top_pinned_titles:
                pinned.append(part)
            elif title == "Submodules":
                submodules_heading.append(part)
            elif title in pinned_h2_titles:
                pinned.append(part)
            elif ".. autofunction::" in part:
                functions.append(part)
            elif ".. automodule::" in part and title.startswith(fqn_prefix):
                short = title[len(fqn_prefix):]
                if short in layout.exported_submodule_shorts:
                    class_sections.append(part)
                else:
                    apidoc_only_submodules.append(part)
            elif ".. autoclass::" in part:
                class_sections.append(part)
            elif ".. automodule::" in part:
                pinned.append(part)
            else:
                pinned.append(part)

        if not class_sections and not apidoc_only_submodules and not functions:
            return content
        if len(class_sections) + len(apidoc_only_submodules) + len(functions) < 2:
            return content

        section_key = lambda block: layout.section_sort_key(
            block.split("\n", 1)[0].strip()
        )

        class_sections.sort(key=section_key)
        apidoc_only_submodules.sort(key=section_key)
        functions.sort(key=section_key)

        merged = preamble + self._join_rst_blocks(
            pinned + class_sections + functions + submodules_heading + apidoc_only_submodules
        )
        if not merged.endswith("\n"):
            merged += "\n"
        return merged

    _RST_SECTION_SPLIT_RE = re.compile(r'(?m)(?=^.+\n[-=~.]+\n)')

    @classmethod
    def _is_document_title_section(cls, part: str, module_name: str) -> bool:
        lines = [line for line in part.strip().splitlines() if line.strip()]
        if len(lines) < 2:
            return False
        return lines[0].strip() == module_name and bool(
            re.fullmatch(r"[=~.]+", lines[1].strip())
        )

    @classmethod
    def _split_rst_sections(
            cls, content: str, module_name: Optional[str] = None) -> Tuple[str, List[str]]:
        parts = cls._RST_SECTION_SPLIT_RE.split(content)
        if len(parts) < 2:
            return content, []
        preamble = parts[0]
        sections = parts[1:]
        if module_name and sections and cls._is_document_title_section(
                sections[0], module_name):
            preamble = preamble + sections[0]
            if not preamble.endswith("\n"):
                preamble += "\n"
            sections = sections[1:]
        return preamble, sections

    @staticmethod
    def _join_rst_blocks(blocks: List[str]) -> str:
        """Join RST section blocks with blank-line separators."""
        parts: List[str] = []
        for block in blocks:
            if not block:
                continue
            normalized = block.rstrip("\n")
            if parts:
                parts.append("\n\n")
            parts.append(normalized)
            parts.append("\n")
        return "".join(parts)

    # ---------------- ouster.sdk toctree ordering ----------------
    _SDK_SUBPACKAGES_ORDER = [
        "ouster.sdk.algorithm",
        "ouster.sdk.core",
        "ouster.sdk.mapping",
        "ouster.sdk.osf",
        "ouster.sdk.pcap",
        "ouster.sdk.perception",
        "ouster.sdk.sensor",
        "ouster.sdk.viz",
        "ouster.sdk.zone_monitor",
        "ouster.sdk.bag",
        "ouster.sdk.util",
        "ouster.sdk.examples",
    ]

    def _reorder_sdk_subpackages(self, content: str) -> str:
        lines = content.splitlines()
        try:
            sub_idx = lines.index("Subpackages")
            submodules_idx = lines.index("Submodules")
        except ValueError:
            return content

        toctree_idx = None
        for i in range(sub_idx, submodules_idx):
            if lines[i].strip().startswith(".. toctree::"):
                toctree_idx = i
                break
        if toctree_idx is None:
            return content

        entries_start = toctree_idx + 1
        # skip option lines and blank lines
        while (entries_start < submodules_idx and
               (lines[entries_start].strip().startswith(":") or
                lines[entries_start].strip() == "")):
            entries_start += 1

        entries_end = submodules_idx
        while entries_end > entries_start and lines[entries_end - 1].strip() == "":
            entries_end -= 1

        new_entries = [""]
        for entry in self._SDK_SUBPACKAGES_ORDER:
            new_entries.append(f"   {entry}")
            new_entries.append("")

        lines[entries_start:entries_end] = new_entries
        return "\n".join(lines) + "\n"

    def _add_sdk_deprecated_page_entry(self, content: str) -> str:
        lines = content.splitlines()
        cleaned: List[str] = []
        i = 0
        while i < len(lines):
            stripped = lines[i].strip()
            if stripped == ".. toctree::":
                block_start = i
                i += 1
                while i < len(lines) and lines[i].strip().startswith(":"):
                    i += 1
                while i < len(lines) and (
                        lines[i].startswith("   ") or lines[i].strip() == ""):
                    i += 1
                block = lines[block_start:i]
                has_page_deprecated = any(
                    line.strip() == "page_deprecated" for line in block
                )
                has_hidden = any(
                    line.strip() == ":hidden:" for line in block
                )
                if has_page_deprecated and has_hidden:
                    cleaned.extend(block)
                    continue
                if has_page_deprecated and not has_hidden:
                    block = [line for line in block if line.strip() != "page_deprecated"]
                cleaned.extend(block)
                continue
            cleaned.append(lines[i])
            i += 1

        if any(
                cleaned[idx].strip() == ".. toctree::"
                and any(
                    line.strip() == ":hidden:"
                    for line in cleaned[idx + 1: min(len(cleaned), idx + 5)]
                )
                and any("page_deprecated" in line for line in cleaned[idx: min(len(cleaned), idx + 12)])
                for idx in range(len(cleaned))):
            return "\n".join(cleaned) + "\n"

        if cleaned and cleaned[-1].strip():
            cleaned.append("")
        cleaned.extend([
            ".. toctree::",
            "   :hidden:",
            "",
            "   page_deprecated",
        ])
        return "\n".join(cleaned) + "\n"

    def _write_deprecations_page(self, api_dir: Path) -> int:
        entries = self._collect_deprecations()
        output = api_dir / "page_deprecated.rst"

        lines: List[str] = [
            "Deprecated List",
            "================",
            "",
        ]

        if not entries:
            lines.extend([
                "No Python deprecations are currently registered.",
                "",
            ])
        else:
            lines.extend([
                ".. list-table::",
                "   :header-rows: 1",
                "   :class: deprecated-list-table",
                "",
                "   * - Deprecated symbol",
                "     - Replacement",
                "     - Last supported",
                "     - Source module",
            ])
            for item in entries:
                lines.extend([
                    f"   * - ``{item['old_symbol']}``",
                    f"     - ``{item['new_symbol']}``",
                    f"     - ``{item['last_supported_version']}``",
                    f"     - ``{item['module']}``",
                ])
            lines.append("")

        output.write_text("\n".join(lines), encoding="utf-8")
        return len(entries)

    def _collect_deprecations(self) -> List[Dict[str, str]]:
        entries = self._collect_python_deprecations()
        entries.extend(self._collect_cpp_binding_deprecations())
        entries.sort(key=lambda item: item["old_symbol"])
        return entries

    def _collect_python_deprecations(self) -> List[Dict[str, str]]:
        src_root = Path(self.app.confdir).parent / "python" / "src" / "ouster"
        if not src_root.exists():
            self.log.warning("Python source path missing for deprecations scan: %s", src_root)
            return []

        entries: List[Dict[str, str]] = []
        seen = set()
        for file_path in src_root.rglob("*.py"):
            try:
                source = file_path.read_text(encoding="utf-8")
                tree = ast.parse(source, filename=str(file_path))
            except Exception as exc:
                self.log.debug("Skipping deprecation scan for %s: %s", file_path, exc)
                continue

            module_name = self._module_name_from_path(src_root, file_path)
            parent_map = self._build_parent_map(tree)
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                if self._is_call_named(node, "deprecated_alias"):
                    old_name = self._extract_call_arg(node, source, 0, "old_name")
                    new_name = self._extract_call_arg(node, source, 1, "new_name")
                    last_supported = self._extract_call_arg(
                        node, source, 4, "last_supported_version")
                    if not old_name or not new_name:
                        continue
                    old_symbol = f"{module_name}.{old_name}" if module_name else old_name
                    new_symbol = f"{module_name}.{new_name}" if module_name else new_name
                elif self._is_call_named(node, "warn_deprecated"):
                    if file_path.name == "_deprecation.py":
                        continue
                    message = self._extract_call_arg(node, source, 0, "message")
                    parsed = self._parse_warn_deprecated_message(message)
                    if not parsed:
                        continue
                    old_name, new_name, last_supported = parsed
                    old_symbol, new_symbol = self._resolve_warn_deprecated_symbols(
                        old_name, new_name, module_name, node, parent_map)
                    if not old_symbol:
                        continue
                else:
                    continue
                last_supported_version = last_supported or deprecation_last_version_default
                unique_key = (old_symbol, new_symbol, last_supported_version)
                if unique_key in seen:
                    continue
                seen.add(unique_key)
                entries.append({
                    "module": module_name,
                    "old_symbol": old_symbol,
                    "new_symbol": new_symbol,
                    "last_supported_version": last_supported_version,
                })

        return entries

    def _collect_cpp_binding_deprecations(self) -> List[Dict[str, str]]:
        src_root = Path(self.app.confdir).parent / "python" / "src" / "cpp"
        if not src_root.exists():
            self.log.warning("C++ source path missing for deprecations scan: %s", src_root)
            return []

        entries: List[Dict[str, str]] = []
        seen = set()
        for file_path in src_root.rglob("*.cpp"):
            source = file_path.read_text(encoding="utf-8")

            module_name = self._module_name_from_path(src_root, file_path)
            public_module_name = self._public_module_name_for_cpp(module_name)
            rel_cpp = str(file_path.relative_to(src_root)).replace("\\", "/")
            for item in self._extract_cpp_deprecations(source, rel_cpp):
                old_symbol = item["old_symbol"]
                unique_key = (public_module_name, old_symbol)
                if unique_key in seen:
                    continue
                seen.add(unique_key)
                entries.append({
                    "module": public_module_name,
                    "old_symbol": old_symbol,
                    "new_symbol": item.get("new_symbol", "unknown"),
                    "last_supported_version": deprecation_last_version_default,
                })

        return entries

    @staticmethod
    def _module_name_from_path(src_root: Path, file_path: Path) -> str:
        rel = file_path.relative_to(src_root)
        parts = list(rel.parts)
        if parts[-1] == "__init__.py":
            parts = parts[:-1]
        else:
            parts[-1] = Path(parts[-1]).stem
        suffix = ".".join(parts)
        return f"ouster.{suffix}" if suffix else "ouster"

    @staticmethod
    def _public_module_name_for_cpp(module_name: str) -> str:
        # C++ bindings are surfaced to users through public ouster.sdk namespaces.
        if module_name.startswith("ouster.client"):
            return "ouster.sdk.core"
        if module_name.startswith("ouster.mapping"):
            return "ouster.sdk.mapping"
        if module_name == "ouster._osf":
            return "ouster.sdk.osf"
        if module_name == "ouster._pcap":
            return "ouster.sdk.pcap"
        if module_name == "ouster._viz":
            return "ouster.sdk.viz"
        if module_name == "ouster._perception":
            return "ouster.sdk.perception"
        return module_name

    @staticmethod
    def _binding_module_for_cpp_rel_path(rel_file: str) -> str:
        rel_norm = rel_file.replace("\\", "/")
        if rel_norm.startswith("client/"):
            return "ouster.sdk._bindings.client"
        if rel_norm.startswith("mapping/"):
            return "ouster.sdk._bindings.mapping"
        name = Path(rel_norm).stem
        if name.startswith("_"):
            return f"ouster.sdk._bindings.{name[1:]}"
        return ""

    @staticmethod
    def _build_parent_map(tree: ast.AST) -> Dict[ast.AST, ast.AST]:
        parent_map: Dict[ast.AST, ast.AST] = {}
        for node in ast.walk(tree):
            for child in ast.iter_child_nodes(node):
                parent_map[child] = node
        return parent_map

    @staticmethod
    def _is_call_named(node: ast.Call, name: str) -> bool:
        func = node.func
        if isinstance(func, ast.Name):
            return func.id == name
        if isinstance(func, ast.Attribute):
            return func.attr == name
        return False

    @staticmethod
    def _parse_warn_deprecated_message(message: str) -> Optional[Tuple[str, str, str]]:
        normalized = " ".join(message.split())
        if not normalized:
            return None

        match = re.search(
            r"^(.+?)\s+is deprecated[:.]?\s+Use\s+(.+?)\s+instead\.\s+"
            r"The last supported version for this will be\s+([\d]+(?:\.[\d]+)*)\.?$",
            normalized,
            re.IGNORECASE,
        )
        if match:
            return match.group(1).strip(), match.group(2).strip(), match.group(3).strip()

        match = re.search(
            r"^(.+?)\s+is deprecated,\s*use\s+(.+?)\s+instead\b",
            normalized,
            re.IGNORECASE,
        )
        if match:
            return match.group(1).strip(), match.group(2).strip(), ""

        match = re.search(
            r"^(.+?)\s+is deprecated,\s*use\s+(.+?)$",
            normalized,
            re.IGNORECASE,
        )
        if match:
            return match.group(1).strip(), match.group(2).strip(), ""

        return None

    @staticmethod
    def _resolve_warn_deprecated_symbols(
            old_name: str, new_name: str, module_name: str,
            node: ast.Call, parent_map: Dict[ast.AST, ast.AST]) -> Tuple[str, str]:
        if old_name.startswith("ouster."):
            return old_name, new_name

        current: Optional[ast.AST] = node
        while current in parent_map:
            current = parent_map[current]
            if isinstance(current, ast.ClassDef):
                return f"{current.name}.{old_name}", f"{current.name}.{new_name}"

        if module_name:
            return f"{module_name}.{old_name}", f"{module_name}.{new_name}"
        return old_name, new_name

    @staticmethod
    def _extract_call_arg(
            node: ast.Call, source: str, position: int, keyword: str) -> str:
        target_node = None
        if len(node.args) > position:
            target_node = node.args[position]
        else:
            for kw in node.keywords:
                if kw.arg == keyword:
                    target_node = kw.value
                    break
        if target_node is None:
            return ""

        if isinstance(target_node, ast.Constant) and isinstance(target_node.value, str):
            return target_node.value

        segment = ast.get_source_segment(source, target_node)
        return segment.strip() if segment else ""

    def _extract_cpp_deprecations(self, source: str, rel_file: str = "") -> List[Dict[str, str]]:
        parser = self._get_cpp_parser()
        source_bytes = source.encode("utf-8")
        tree = parser.parse(source_bytes)
        call_nodes = sorted(
            self._iter_nodes_by_type(tree.root_node, "call_expression"),
            key=lambda node: (node.start_byte, node.end_byte),
        )
        binding_module = self._binding_module_for_cpp_rel_path(rel_file)

        entries: List[Dict[str, str]] = []
        for warn_call in call_nodes:
            if not self._is_deprecation_warn_call(warn_call, source_bytes):
                continue

            line_num = warn_call.start_point[0] + 1
            binding_call = self._find_enclosing_binding_call(warn_call, source_bytes)
            binding = self._extract_binding_context(binding_call, source_bytes)

            warn_message = self._extract_warn_message(warn_call, source_bytes)
            old_text, replacement_text = self._extract_symbols_from_warn_message(warn_message)
            if not replacement_text:
                raise RuntimeError(
                    f"Could not infer replacement symbol for deprecated API at {rel_file}:{line_num}.\n"
                    f"old_symbol={old_text}\n"
                    f"warning_text={warn_message!r}\n"
                    "Update the C++ warning message to: "
                    "'<old> is deprecated, use <new_symbol> instead'.")

            old_symbol = self._render_symbol_from_binding(
                binding, binding_module, rel_file, line_num, fallback_symbol=old_text
            )
            if not old_symbol:
                raise RuntimeError(
                    f"Could not infer deprecated symbol from binding context at {rel_file}:{line_num}.\n"
                    "Update the pybind/nanobind binding so the deprecated warning is inside a "
                    "clear .def/.def_static/module.def call with a named binding.")

            replacement_binding = self._resolve_replacement_binding_call(
                replacement_text, binding_call, call_nodes, source_bytes, rel_file, line_num
            )
            if replacement_binding:
                replacement_ctx = self._extract_binding_context(
                    replacement_binding, source_bytes)
                new_symbol = self._render_symbol_from_binding(
                    replacement_ctx, binding_module, rel_file, line_num, fallback_symbol=replacement_text
                )
            else:
                new_symbol = self._callable_name_from_symbol_text(replacement_text)

            entries.append({"old_symbol": old_symbol, "line": str(line_num), "new_symbol": new_symbol})

        return entries

    def _get_cpp_parser(self) -> "Parser":
        if self._cpp_parser:
            return self._cpp_parser
        try:
            from tree_sitter import Language, Parser  # type: ignore[import-not-found]
            import tree_sitter_cpp  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportError(_CPP_PARSER_DEPS_MSG) from exc
        # tree-sitter ≥ 0.22 should not have mypy issues, mypy stubs are on older version.
        language = Language(tree_sitter_cpp.language())  # type: ignore[call-arg]
        self._cpp_parser = Parser(language)  # type: ignore[call-arg]
        return self._cpp_parser

    def _iter_nodes_by_type(self, root: "Node", node_type: str) -> List["Node"]:
        nodes: List["Node"] = []
        stack = [root]
        while stack:
            node = stack.pop()
            if node.type == node_type:
                nodes.append(node)
            stack.extend(reversed(node.children))
        return nodes

    def _node_text(self, node: Optional["Node"], source_bytes: bytes) -> str:
        if not node:
            return ""
        return source_bytes[node.start_byte:node.end_byte].decode("utf-8", "ignore")

    def _is_deprecation_warn_call(self, call_node: "Node", source_bytes: bytes) -> bool:
        func = call_node.child_by_field_name("function")
        if self._node_text(func, source_bytes).strip() != "PyErr_WarnEx":
            return False
        args = call_node.child_by_field_name("arguments")
        if not args or len(args.named_children) < 2:
            return False
        first_arg = self._node_text(args.named_children[0], source_bytes).strip()
        return first_arg == "PyExc_FutureWarning"

    def _find_enclosing_binding_call(self, node: "Node", source_bytes: bytes) -> Optional["Node"]:
        current = node.parent
        while current:
            if current.type == "call_expression" and self._is_cpp_binding_call(current, source_bytes):
                return current
            current = current.parent
        return None

    def _is_cpp_binding_call(self, call_node: "Node", source_bytes: bytes) -> bool:
        func_text = self._node_text(call_node.child_by_field_name("function"), source_bytes)
        return bool(re.search(r"\.def_static\b", func_text) or
                    re.search(r"\.def\b", func_text) or
                    re.search(r"\bmodule\.def\b", func_text))

    def _find_next_binding_call(
            self, call_nodes: List["Node"], current_binding: Optional["Node"],
            source_bytes: bytes) -> Optional["Node"]:
        if not current_binding:
            return None
        # In chained bindings like .def(...).def(...), tree-sitter nests call
        # expressions with the same start byte. The immediate next binding is
        # the first parent call_expression that is also a binding call.
        current = current_binding.parent
        while current:
            if current.type == "call_expression" and self._is_cpp_binding_call(current, source_bytes):
                return current
            current = current.parent

        current_pos = (current_binding.start_byte, current_binding.end_byte)
        for node in call_nodes:
            node_pos = (node.start_byte, node.end_byte)
            if node_pos <= current_pos:
                continue
            if self._is_cpp_binding_call(node, source_bytes):
                return node
        return None

    def _extract_binding_context(
            self, binding_call: Optional["Node"], source_bytes: bytes) -> Dict[str, str]:
        if not binding_call:
            return {"name": "", "class_name": ""}
        func_text = self._node_text(binding_call.child_by_field_name("function"), source_bytes)
        class_match = self._CPP_CLASS_RE.search(func_text)
        class_name = class_match.group(1) if class_match else ""

        args = binding_call.child_by_field_name("arguments")
        binding_name = ""
        py_sig = ""
        if args:
            for idx, child in enumerate(args.named_children):
                if idx == 0:
                    binding_name = self._extract_string_literal(child, source_bytes)
                    if not binding_name and class_name:
                        first_arg_text = self._node_text(child, source_bytes).strip()
                        if "py::init<" in first_arg_text:
                            binding_name = "__init__"
                elif child.type == "call_expression":
                    fn_text = self._node_text(
                        child.child_by_field_name("function"), source_bytes)
                    if fn_text in ("py::sig", "nb::sig"):
                        sig_args = child.child_by_field_name("arguments")
                        if sig_args and sig_args.named_children:
                            py_sig = self._extract_string_literal(
                                sig_args.named_children[0], source_bytes)
        return {"name": binding_name, "class_name": class_name, "py_sig": py_sig}

    def _extract_warn_message(self, warn_call: "Node", source_bytes: bytes) -> str:
        args = warn_call.child_by_field_name("arguments")
        if not args or len(args.named_children) < 2:
            return ""
        return self._extract_string_literal(args.named_children[1], source_bytes)

    def _extract_symbols_from_warn_message(self, message: str) -> Tuple[str, str]:
        normalized = " ".join(message.split())
        match = re.search(r"^(.+?)\s+is deprecated,\s*use\s+(.+?)\s+instead\b", normalized)
        if not match:
            return "", ""

        def _strip_quotes(s: str) -> str:
            """Strip enclosing single or double quotes"""
            if len(s) >= 2 and s[0] == s[-1] and s[0] in ("'", '"'):
                return s[1:-1]
            return s

        return match.group(1).strip(), _strip_quotes(match.group(2).strip())

    def _resolve_replacement_binding_call(
            self, replacement_text: str, binding_call: Optional["Node"],
            call_nodes: List["Node"], source_bytes: bytes,
            rel_file: str, line_num: int) -> Optional["Node"]:
        next_binding = self._find_next_binding_call(call_nodes, binding_call, source_bytes)
        if next_binding:
            next_ctx = self._extract_binding_context(next_binding, source_bytes)
            if (self._binding_matches_symbol_text(next_ctx, replacement_text)
                    and self._binding_matches_replacement_arg_count(
                        next_binding, replacement_text, source_bytes)):
                return next_binding

        chain_binding = self._find_matching_binding_in_same_chain(
            binding_call, replacement_text, source_bytes, rel_file)
        if chain_binding:
            return chain_binding

        self.log.warning(
            "Could not resolve replacement binding at %s:%s (replacement_text=%r); using name only.",
            rel_file, line_num, replacement_text)
        return None

    def _find_matching_binding_in_same_chain(
            self, binding_call: Optional["Node"], replacement_text: str,
            source_bytes: bytes, rel_file: str) -> Optional["Node"]:
        if not binding_call:
            return None
        fn_node = binding_call.child_by_field_name("function")
        if not fn_node:
            return None

        chain_calls = sorted(
            [node for node in self._iter_nodes_by_type(fn_node, "call_expression")
             if self._is_cpp_binding_call(node, source_bytes)],
            key=lambda node: (node.start_byte, node.end_byte),
        )

        matches: List["Node"] = []
        for call_node in chain_calls:
            ctx = self._extract_binding_context(call_node, source_bytes)
            if self._binding_matches_symbol_text(ctx, replacement_text):
                matches.append(call_node)

        matches = self._filter_bindings_by_replacement_arg_count(
            matches, replacement_text, source_bytes)
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            candidate_names = [
                self._binding_callable_name(
                    self._extract_binding_context(node, source_bytes))
                for node in matches
            ]
            raise RuntimeError(
                f"Ambiguous replacement in same binding chain for {rel_file}.\n"
                f"replacement_text={replacement_text!r}\n"
                f"candidates={candidate_names!r}")
        return None

    def _count_binding_py_args(self, binding_call: "Node", source_bytes: bytes) -> int:
        args = binding_call.child_by_field_name("arguments")
        if not args:
            return 0
        return sum(
            1 for child in args.named_children
            if re.search(r"\b(?:py|nb)::arg\b", self._node_text(child, source_bytes))
        )

    def _filter_bindings_by_replacement_arg_count(
            self, matches: List["Node"], replacement_text: str,
            source_bytes: bytes) -> List["Node"]:
        _, replacement_args, replacement_has_call = self._parse_symbol_text(replacement_text)
        if len(matches) <= 1 or not replacement_has_call:
            return matches
        expected = len(replacement_args)
        filtered = [
            node for node in matches
            if self._count_binding_py_args(node, source_bytes) == expected
        ]
        return filtered or matches

    def _binding_matches_replacement_arg_count(
            self, binding_call: "Node", replacement_text: str,
            source_bytes: bytes) -> bool:
        return len(self._filter_bindings_by_replacement_arg_count(
            [binding_call], replacement_text, source_bytes)) == 1

    def _binding_matches_symbol_text(self, binding: Dict[str, str], symbol_text: str) -> bool:
        symbol_name = self._callable_name_from_symbol_text(symbol_text)
        binding_name = self._binding_callable_name(binding)
        if not symbol_name or not binding_name:
            return False
        if "." in symbol_name:
            return symbol_name == binding_name
        return symbol_name == binding_name.split(".")[-1]

    def _callable_name_from_symbol_text(self, value: str) -> str:
        name, _, _ = self._parse_symbol_text(value)
        return name

    def _binding_callable_name(self, binding: Dict[str, str]) -> str:
        class_name = binding.get("class_name", "")
        binding_name = binding.get("name", "")
        if class_name and binding_name == "__init__":
            return class_name
        if class_name and binding_name:
            return f"{class_name}.{binding_name}"
        return binding_name

    def _binding_qualname_for_stub(self, binding: Dict[str, str]) -> str:
        class_name = binding.get("class_name", "")
        binding_name = binding.get("name", "")
        if class_name and binding_name == "__init__":
            return f"{class_name}.__init__"
        if class_name and binding_name:
            return f"{class_name}.{binding_name}"
        return binding_name

    @staticmethod
    def _parse_pysig_params(py_sig: str) -> Optional[Dict[str, str]]:
        """Parse 'def name(self, *, foo: T, bar: U) -> R' into {'foo': 'T', 'bar': 'U'}.

        Splits by commas at bracket-depth 0 so generic types like
        Sequence[Optional[LidarFrame]] are not broken up.
        """
        normalized = " ".join(py_sig.split())
        m = re.match(r"def\s+\w+\s*\((.+)\)(?:\s*->.*)?$", normalized)
        if not m:
            return None
        params_str = m.group(1)
        # Bracket-depth-aware comma split
        tokens: List[str] = []
        depth = 0
        buf: List[str] = []
        for ch in params_str:
            if ch in "([{":
                depth += 1
                buf.append(ch)
            elif ch in ")]}":
                depth -= 1
                buf.append(ch)
            elif ch == "," and depth == 0:
                tokens.append("".join(buf).strip())
                buf = []
            else:
                buf.append(ch)
        if buf:
            tokens.append("".join(buf).strip())
        result: Dict[str, str] = {}
        for token in tokens:
            # Strip keyword-only / positional-only markers and skip self/cls
            p = token.lstrip("*").lstrip("/").strip()
            if not p or p in ("self", "cls"):
                continue
            if ":" in p:
                name, type_str = p.split(":", 1)
                name = name.split("=")[0].strip()  # drop default value if any
                if name:
                    result[name] = type_str.strip()
        return result if result else None

    def _render_symbol_from_binding(
            self, binding: Dict[str, str], binding_module: str,
            rel_file: str, line_num: int, fallback_symbol: str = "") -> str:
        display_name = self._binding_callable_name(binding) or self._callable_name_from_symbol_text(
            fallback_symbol)
        if not display_name:
            return ""

        qualname = self._binding_qualname_for_stub(binding)
        if not binding_module or not qualname:
            if fallback_symbol:
                self.log.warning(
                    "Missing binding module/qualname for %s:%s; using name-only symbol %r",
                    rel_file, line_num, display_name)
            return display_name

        py_sig = binding.get("py_sig", "")
        pysig_params = self._parse_pysig_params(py_sig) if py_sig else None
        if pysig_params is not None:
            param_types = pysig_params
        else:
            param_types = get_stub_param_types(binding_module, qualname)
        if not param_types:
            self.log.warning(
                "No stub signature for %s (%s:%s); using name-only symbol %r",
                qualname, rel_file, line_num, display_name)
            return display_name

        _, fallback_args, fallback_has_call = self._parse_symbol_text(fallback_symbol)
        param_items = list(param_types.items())
        rendered_params: List[str] = []
        if fallback_has_call and not fallback_args:
            return f"{display_name}()"

        if fallback_args:
            if any(arg in param_types for arg in fallback_args):
                for arg in fallback_args:
                    if arg in param_types:
                        rendered_params.append(f"{arg}: {param_types[arg]}")
                    else:
                        rendered_params.append(arg)
            elif len(fallback_args) <= len(param_items):
                start = max(0, len(param_items) - len(fallback_args))
                for idx, arg in enumerate(fallback_args):
                    rendered_params.append(f"{arg}: {param_items[start + idx][1]}")
            else:
                rendered_params = [f"{name}: {type_name}" for name, type_name in param_items]
        else:
            rendered_params = [f"{name}: {type_name}" for name, type_name in param_items]

        params = ", ".join(rendered_params)
        return f"{display_name}({params})"

    def _parse_symbol_text(self, value: str) -> Tuple[str, List[str], bool]:
        text = " ".join(value.split()).strip()
        if not text:
            return "", [], False
        if "=" in text:
            text = text.split("=", 1)[1].strip()
        match = re.match(r"([A-Za-z_][\w\.]*)\s*(?:\((.*)\))?$", text)
        if not match:
            return text, [], False
        name = match.group(1)
        args_raw = match.group(2)
        has_call = args_raw is not None
        if not args_raw or not args_raw.strip():
            return name, [], has_call
        arg_names: List[str] = []
        for raw_arg in args_raw.split(","):
            arg = raw_arg.strip()
            if not arg:
                continue
            arg = arg.split(":", 1)[0].strip()
            if " " in arg:
                arg = arg.rsplit(" ", 1)[-1]
            arg_names.append(arg.strip("&* "))
        return name, arg_names, has_call

    def _extract_string_literal(self, node: Optional["Node"], source_bytes: bytes) -> str:
        text = self._node_text(node, source_bytes)
        if not text:
            return ""
        literals = re.findall(r'"(?:\\.|[^"\\])*"', text)
        if not literals:
            return ""
        pieces: List[str] = []
        for lit in literals:
            try:
                value = ast.literal_eval(lit)
            except Exception:
                value = lit.strip('"')
            pieces.append(value)
        return "".join(pieces)
