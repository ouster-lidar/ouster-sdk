"""
RST processor: single pass over generated ouster.sdk*.rst files.
  - Adds pybind sections
"""
from pathlib import Path
import re
from typing import Dict, List
from util import configure_logger
from pybind import discover_pybind


class RstProcessor:
    def __init__(self, app):
        self.app = app
        self.log = configure_logger("ouster.docs.rstproc")

    def process_all(self):
        api_dir = Path(self.app.confdir) / "python" / "api_python"
        rst_files = list(api_dir.glob("ouster.sdk*.rst"))
        self.log.info(f"Processing {len(rst_files)} RST files in single pass")

        stats = {"modified": 0, "skipped": 0, "pybind_added": 0}

        for rst_file in rst_files:
            original = rst_file.read_text(encoding="utf-8")
            module_name = rst_file.stem
            modified = self._transform_rst_content(original, module_name, stats)

            if modified != original:
                rst_file.write_text(modified, encoding="utf-8")
                stats["modified"] += 1
                self.log.debug(f"Modified {module_name}")
            else:
                stats["skipped"] += 1

        self.log.info(f"RST processing complete: {stats}")

    def _transform_rst_content(self, content: str, module_name: str, stats: Dict) -> str:
        # Pybind sections
        if (module_name != "ouster.sdk"
                and not module_name.endswith("._utils")):
            new_content = self._add_pybind_sections(content, module_name)
            if new_content != content:
                content = new_content
                stats["pybind_added"] += 1
                self.log.debug(f"Added pybind sections to {module_name}")
        return content

    # ---------------- Pybind Sections ----------------
    def _add_pybind_sections(self, content: str, module_name: str) -> str:
        data = discover_pybind(self.app, module_name)
        if not data["classes"] and not data["functions"]:
            return content

        lines: List[str] = []
        if data["classes"]:
            for cls in data["classes"]:
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
            for fn in data["functions"]:
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