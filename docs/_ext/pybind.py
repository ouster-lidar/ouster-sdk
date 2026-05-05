"""
Parse package __init__.py for re-exports from ouster.sdk._bindings.*
"""
import importlib
import inspect
import ast
from pathlib import Path
from typing import Dict, List
from util import configure_logger

log = configure_logger("ouster.docs.pybind")


def discover_pybind(app, module_name: str) -> Dict[str, list]:
    try:
        reexports = _find_binding_imports(app, module_name)
        namespace = _get_module_objects(module_name)

        classes = []
        functions = []

        for public_name, src_binding in reexports:
            obj = namespace.get(public_name)
            if obj is None:
                continue
            if not _is_binding(obj):
                continue

            fq = f"{module_name}.{public_name}"
            if inspect.isclass(obj):
                classes.append({
                    "name": fq,
                    "source_binding": src_binding
                })
            elif callable(obj):
                functions.append({
                    "name": fq,
                    "source_binding": src_binding
                })

        return {"classes": classes, "functions": functions}
    except Exception as e:
        log.debug(f"Pybind discovery failed for {module_name}: {e}")
        return {"classes": [], "functions": []}


def _find_binding_imports(app, module_name: str) -> List[tuple]:
    """
    Parse __init__.py with the AST module to find re-exports from ouster.sdk._bindings.*
    Returns a list of (public_name, source_binding_module) tuples.
    """
    if not module_name.startswith("ouster.sdk"):
        return []
    parts = module_name.split(".")
    if len(parts) < 2:
        return []
    base_path = Path(app.confdir).parent / "python" / "src"
    init_path = base_path / "/".join(parts) / "__init__.py"
    if not init_path.exists():
        return []

    results: List[tuple] = []
    try:
        content = init_path.read_text(encoding="utf-8")
    except Exception as e:
        log.debug(f"Could not read {init_path}: {e}")
        return []

    try:
        tree = ast.parse(content, filename=str(init_path))
    except Exception as e:
        log.debug(f"Could not parse {init_path}: {e}")
        return []

    # Go through import statements in AST
    for node in ast.iter_child_nodes(tree):
        # Only look for "from ... import ..." statements
        if isinstance(node, ast.ImportFrom):
            mod = node.module
            if mod is None:
                continue
            # e.g., "ouster.sdk._bindings.foo"
            if mod.startswith("ouster.sdk._bindings."):
                submod = mod[len("ouster.sdk._bindings."):]
                for alias in node.names:
                    public_name = alias.asname if alias.asname else alias.name
                    results.append((public_name, f"ouster.sdk._bindings.{submod}"))
    return results


def _get_module_objects(module_name: str):
    try:
        mod = importlib.import_module(module_name)
    except Exception as e:
        log.debug(f"Import failed for {module_name}: {e}")
        return {}
    out = {}
    for n in dir(mod):
        if n.startswith("_"):
            continue
        try:
            out[n] = getattr(mod, n)
        except Exception:
            pass
    return out


def _is_binding(obj) -> bool:
    return getattr(obj, "__module__", "").startswith("ouster.sdk._bindings")