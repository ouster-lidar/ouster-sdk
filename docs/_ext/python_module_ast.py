"""
Shared AST analysis of ouster.sdk package __init__.py files for docs build.

Parse once per package, cache on app.env, consume from nanobind, rst_processor,
and ouster_post_build.
"""
from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

from util import configure_logger

log = configure_logger("ouster.docs.python_ast")

_BINDINGS_PREFIX = "ouster.sdk._bindings."


@dataclass(frozen=True)
class PublicExport:
    name: str
    source_submodule: str | None
    binding_module: str | None


@dataclass(frozen=True)
class PackageAnalysis:
    module_name: str
    public_exports: Tuple[PublicExport, ...]

    @property
    def binding_reexports(self) -> Tuple[Tuple[str, str], ...]:
        return tuple(
            (export.name, export.binding_module)
            for export in self.public_exports
            if export.binding_module
        )

    @property
    def exported_submodule_shorts(self) -> frozenset[str]:
        return frozenset(
            export.source_submodule
            for export in self.public_exports
            if export.source_submodule
        )

    def exports_for_submodule(self, short: str) -> Tuple[str, ...]:
        return tuple(
            export.name
            for export in self.public_exports
            if export.source_submodule == short
        )

    def section_sort_key(self, title: str) -> str:
        """Sort key for an RST h2 exported class name."""
        prefix = self.module_name + "."
        if title.startswith(prefix):
            short = title[len(prefix):]
            names = self.exports_for_submodule(short)
            if names:
                return min(name.casefold() for name in names)
            return short.casefold()
        return title.casefold()


def package_src_root(app) -> Path:
    return Path(app.confdir).parent / "python" / "src"


def package_init_path(app, module_name: str) -> Optional[Path]:
    if not module_name.startswith("ouster.sdk"):
        return None
    parts = module_name.split(".")
    if len(parts) < 2:
        return None
    init_path = package_src_root(app) / "/".join(parts) / "__init__.py"
    if not init_path.is_file():
        return None
    return init_path


def parse_package_init(app, module_name: str) -> Optional[ast.Module]:
    init_path = package_init_path(app, module_name)
    if init_path is None:
        return None
    try:
        content = init_path.read_text(encoding="utf-8")
    except OSError as exc:
        log.error("Could not read %s: %s", init_path, exc)
        return None
    try:
        return ast.parse(content, filename=str(init_path))
    except SyntaxError as exc:
        log.error("Could not parse %s: %s", init_path, exc)
        return None


def _analyze_tree(module_name: str, tree: ast.Module) -> PackageAnalysis:
    exports: List[PublicExport] = []
    for node in ast.iter_child_nodes(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        mod = node.module
        names = [alias.asname or alias.name for alias in node.names]
        if node.level == 0 and mod and mod.startswith(_BINDINGS_PREFIX):
            for name in names:
                exports.append(PublicExport(name, None, mod))
        elif node.level > 0 and mod:
            for name in names:
                exports.append(PublicExport(name, mod, None))
    return PackageAnalysis(module_name, tuple(exports))


def analyze_package(app, module_name: str) -> PackageAnalysis:
    tree = parse_package_init(app, module_name)
    if tree is None:
        return PackageAnalysis(module_name=module_name, public_exports=())
    return _analyze_tree(module_name, tree)


def get_package_analysis(app, module_name: str) -> PackageAnalysis:
    cache = getattr(app.env, "ouster_package_analysis", None)
    if cache is None:
        cache = {}
        app.env.ouster_package_analysis = cache
    if module_name not in cache:
        init_path = package_init_path(app, module_name)
        analysis = analyze_package(app, module_name)
        cache[module_name] = analysis
        if init_path is None:
            log.info(
                "%s: no source __init__.py under python/src; "
                "binding reexports=%d",
                module_name,
                len(analysis.binding_reexports),
            )
        else:
            log.info(
                "%s: parsed AST from %s; binding reexports=%d submodule exports=%d",
                module_name,
                init_path,
                len(analysis.binding_reexports),
                len(analysis.exported_submodule_shorts),
            )
            if analysis.binding_reexports:
                log.debug(
                    "%s: AST binding names: %s",
                    module_name,
                    [name for name, _ in analysis.binding_reexports],
                )
    return cache[module_name]
