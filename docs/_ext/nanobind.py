"""
Pybind/nanobind binding support for docs.

- Discover re-exports from ouster.sdk._bindings.* (C++ bindings from python/src/cpp)
  by parsing package __init__.py.
- Docstring enrichment (autodoc-process-docstring): only for those bindings, inject
  Args types from the function signature when the docstring omits them.
"""
import importlib
import inspect
import ast
import enum
import re
from functools import lru_cache
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from python_module_ast import get_package_analysis
from util import configure_logger

log = configure_logger("ouster.docs.nb")


def discover_nb(app, module_name: str) -> Dict[str, list]:
    try:
        reexports = _find_binding_imports(app, module_name)
        namespace, module_file = _get_module_objects(module_name)

        classes = []
        functions = []
        missing_from_runtime: List[str] = []
        skipped_not_binding: List[str] = []

        for public_name, src_binding in reexports:
            obj = namespace.get(public_name)
            if obj is None:
                missing_from_runtime.append(public_name)
                continue
            if not _is_binding(obj):
                skipped_not_binding.append(public_name)
                continue
            # Only apply stub metadata to classes; skip instances/constants (e.g. Version instance, int)
            if inspect.isclass(obj):
                _apply_stub_metadata(obj, src_binding)

            fq = f"{module_name}.{public_name}"
            if inspect.isclass(obj):
                classes.append({
                    "name": fq,
                    "source_binding": src_binding,
                    "is_enum": _is_enum(obj)
                })
            elif callable(obj):
                functions.append({
                    "name": fq,
                    "source_binding": src_binding
                })

        runtime_bindings = sorted(
            name for name, obj in namespace.items() if _is_binding(obj)
        )
        if missing_from_runtime:
            log.warning(
                "%s: %d AST binding name(s) missing at runtime (imported from %s): %s",
                module_name,
                len(missing_from_runtime),
                module_file or "<unknown>",
                missing_from_runtime,
            )
            if runtime_bindings:
                log.warning(
                    "%s: runtime binding exports available but not in AST: %s",
                    module_name,
                    runtime_bindings,
                )
        if skipped_not_binding:
            log.debug(
                "%s: skipped non-binding AST names: %s",
                module_name,
                skipped_not_binding,
            )

        log.info(
            "%s: discover_nb found %d classes, %d functions "
            "(AST reexports=%d, runtime module=%s)",
            module_name,
            len(classes),
            len(functions),
            len(reexports),
            module_file or "<import failed>",
        )
        return {"classes": classes, "functions": functions}
    except Exception as e:
        log.error(f"Pybind discovery failed for {module_name}: {e}", exc_info=True)
        return {"classes": [], "functions": []}


def _find_binding_imports(app, module_name: str) -> List[tuple]:
    """
    Re-exports from ouster.sdk._bindings.* discovered via shared package AST.
    Returns a list of (public_name, source_binding_module) tuples.
    """
    if not module_name.startswith("ouster.sdk"):
        return []
    return list(get_package_analysis(app, module_name).binding_reexports)


def _get_module_objects(module_name: str):
    try:
        mod = importlib.import_module(module_name)
    except SystemExit as e:
        log.warning(f"Import of {module_name} called exit({e.code}); skipping")
        return {}, None
    except Exception as e:
        log.error(f"Import failed for {module_name}: {e}")
        return {}, None
    module_file = getattr(mod, "__file__", None)
    log.debug("Imported %s from %s", module_name, module_file)
    out = {}
    for n in dir(mod):
        if n.startswith("_"):
            continue
        try:
            out[n] = getattr(mod, n)
        except Exception:
            pass
    return out, module_file


def _is_binding(obj) -> bool:
    mod = getattr(obj, "__module__", None) or ""
    return isinstance(mod, str) and mod.startswith("ouster.sdk._bindings")


def _is_enum(obj) -> bool:
    try:
        if issubclass(obj, enum.Enum):
            return True
    except Exception:
        pass
    return hasattr(obj, "__members__")


def _apply_stub_metadata(obj, binding_module: str):
    if not inspect.isclass(obj):
        return

    try:
        annotations = _load_stub_annotations(binding_module).get(obj.__name__)
        if not annotations:
            log.debug(f"No stub annotations for {obj.__module__}.{obj.__name__}")
            return
        existing = dict(getattr(obj, "__annotations__", {}))
        for attr, type_str in annotations.items():
            existing.setdefault(attr, type_str)
            descriptor = getattr(obj, attr, None)
            if descriptor is None:
                log.debug(f"Descriptor missing for {obj.__name__}.{attr}")
                continue

            # Replace pybind property descriptors with Python properties so we can attach annotations.
            if isinstance(descriptor, property):
                orig_getter = descriptor.fget
                orig_setter = descriptor.fset
                orig_deleter = descriptor.fdel

                def _wrap_getter(getter, _type_str=type_str, _attr=attr):
                    if getter is None:
                        return None

                    def wrapped(self, _getter=getter):
                        return _getter(self)

                    wrapped.__annotations__ = {"return": _type_str}
                    if getter.__doc__:
                        wrapped.__doc__ = getter.__doc__
                    wrapped.__name__ = f"{_attr}_getter"
                    return wrapped

                def _wrap_setter(setter, _attr=attr):
                    if setter is None:
                        return None

                    def wrapped(self, value, _setter=setter):
                        return _setter(self, value)

                    wrapped.__name__ = f"{_attr}_setter"
                    return wrapped

                def _wrap_deleter(deleter, _attr=attr):
                    if deleter is None:
                        return None

                    def wrapped(self, _deleter=deleter):
                        return _deleter(self)

                    wrapped.__name__ = f"{_attr}_deleter"
                    return wrapped
                new_prop = property(
                    _wrap_getter(orig_getter, type_str),
                    _wrap_setter(orig_setter),
                    _wrap_deleter(orig_deleter),
                    descriptor.__doc__
                )
                setattr(obj, attr, new_prop)
                descriptor = new_prop

            # Leave the docstring untouched; signature type will come from the wrapper.

        obj.__annotations__ = existing
    except Exception as exc:
        log.warning(
            f"Failed to apply stub annotations for {obj.__module__}.{obj.__name__}: {exc}"
        )


@lru_cache(maxsize=None)
def _load_stub_annotations(binding_module: str) -> Dict[str, Dict[str, str]]:
    stub_path = _stub_path_for_binding_module(binding_module)
    if not stub_path:
        return {}
    try:
        tree = ast.parse(stub_path.read_text(encoding="utf-8"), filename=str(stub_path))
    except Exception:
        return {}

    annotations: Dict[str, Dict[str, str]] = {}
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            class_ann = _collect_class_annotations(node)
            if class_ann:
                annotations[node.name] = class_ann
    return annotations


def _collect_class_annotations(class_node: ast.ClassDef) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for stmt in class_node.body:
        if isinstance(stmt, ast.AnnAssign) and isinstance(stmt.target, ast.Name):
            try:
                out[stmt.target.id] = ast.unparse(stmt.annotation)  # type: ignore[attr-defined]  # Python 3.9+
            except Exception:
                continue
        elif isinstance(stmt, ast.FunctionDef) and _is_property(stmt):
            if stmt.returns is None:
                continue
            try:
                out[stmt.name] = ast.unparse(stmt.returns)  # type: ignore[attr-defined]  # Python 3.9+
            except Exception:
                continue
    if out:
        log.info(f"Collected annotations for class {class_node.name}: {out}")
    return out


def _is_property(func_def: ast.FunctionDef) -> bool:
    for decorator in func_def.decorator_list:
        if isinstance(decorator, ast.Name) and decorator.id == "property":
            return True
        if isinstance(decorator, ast.Attribute) and decorator.attr == "setter":
            return True
    return False


def _stub_path_for_binding_module(binding_module: str) -> Optional[Path]:
    """Resolve .pyi path for ouster.sdk._bindings.<name> via ouster.sdk __file__ -> parent/_bindings/<name>.pyi."""
    if not binding_module.startswith("ouster.sdk._bindings."):
        return None
    name = binding_module.split(".")[-1]
    try:
        parent_mod = importlib.import_module("ouster.sdk")
        parent_file = getattr(parent_mod, "__file__", None)
        if not parent_file:
            return None
        p = Path(parent_file).resolve().parent / "_bindings" / (name + ".pyi")
        return p if p.exists() else None
    except Exception:
        return None


def _get_param_types_from_stub(
    binding_module: str, qualname: str
) -> Dict[str, str]:
    """Extract param name -> type from a .pyi stub (merge all overloads)."""
    stub_path = _stub_path_for_binding_module(binding_module)
    if not stub_path:
        return {}
    try:
        tree = ast.parse(stub_path.read_text(encoding="utf-8"), filename=str(stub_path))
    except Exception as exc:
        log.debug("_get_param_types_from_stub: parse failed %s: %s", stub_path, exc)
        return {}
    parts = qualname.split(".", 1)
    if len(parts) == 1:
        for node in tree.body:
            if isinstance(node, ast.FunctionDef) and node.name == parts[0]:
                return _param_annotations_from_ast_func(node)
        return {}
    class_name, method_name = parts[0], parts[1]
    merged: Dict[str, str] = {}
    for node in tree.body:
        if not isinstance(node, ast.ClassDef) or node.name != class_name:
            continue
        for stmt in node.body:
            if isinstance(stmt, ast.FunctionDef) and stmt.name == method_name:
                for k, v in _param_annotations_from_ast_func(stmt).items():
                    merged.setdefault(k, v)
        return merged
    return {}


def _param_annotations_from_ast_func(func_node: ast.FunctionDef) -> Dict[str, str]:
    """Get param name -> type string from an ast.FunctionDef (skip self/cls).
    Include posonlyargs (params before /), args (positional-or-keyword), and kwonlyargs.
    """
    out: Dict[str, str] = {}

    def add_from_arg_list(arg_list):
        for arg in arg_list:
            if arg.arg in ("self", "cls"):
                continue
            if arg.annotation is None:
                continue
            try:
                type_str = ast.unparse(arg.annotation)
                if type_str:
                    out[arg.arg] = type_str
            except Exception:
                continue

    # Params before / are in posonlyargs (e.g. def add_field(self, name: str, ..., /))
    add_from_arg_list(getattr(func_node.args, "posonlyargs", []))
    add_from_arg_list(func_node.args.args)
    add_from_arg_list(getattr(func_node.args, "kwonlyargs", []))
    return out


def get_stub_param_types(binding_module: str, qualname: str) -> Dict[str, str]:
    """Public helper for stub-first parameter type lookup."""
    return _get_param_types_from_stub(binding_module, qualname)


# ---------------------------------------------------------------------------
# Docstring Args enrichment: inject parameter types from function signature
# ---------------------------------------------------------------------------

def get_signature_param_types(obj) -> Dict[str, str]:
    """
    Get parameter name -> type string. Used to fill Args types
    when the docstring omits them.

    For nanobind bindings (ouster.sdk._bindings.*), inspect.signature() fails;
    we use the installed .pyi stubs as the primary source. Runtime signature
    is only used when not a binding or when stub has no types for this callable.
    """
    binding_module = getattr(obj, "__module__", None) or ""
    qualname = getattr(obj, "__qualname__", None) or getattr(obj, "__name__", "")
    is_binding = _is_binding(obj)

    if is_binding and qualname:
        stub_types = _get_param_types_from_stub(binding_module, qualname)
        if stub_types:
            return stub_types

    # Runtime signature (bindings without stub coverage)
    param_types: Dict[str, str] = {}
    try:
        sig = inspect.signature(obj)
        for name, param in sig.parameters.items():
            if name in ("self", "cls"):
                continue
            if param.annotation is inspect.Parameter.empty:
                continue
            try:
                type_str = _format_annotation(param.annotation)
                if type_str:
                    param_types[name] = type_str
            except Exception:
                pass
    except Exception:
        pass
    return param_types


def _format_annotation(ann) -> str:
    """Turn a type annotation into a short string for docstrings."""
    if ann is None or ann is inspect.Parameter.empty:
        return ""
    if isinstance(ann, type):
        return ann.__name__
    s = str(ann)
    # Prefer short form for typing.*
    if "typing." in s:
        s = re.sub(r"typing\.(\w+)", r"\1", s)
    return s


# Match an Args line: optional leading indent, param name, optional (type), colon, rest.
# e.g. "        hostname (str): hostname of the sensor" or "        hostname): ..."
_ARGS_LINE_RE = re.compile(r"^(\s*)(\w+)\s*(?:\(([^)]*)\))?\s*:\s*(.*)$")

# Sphinx :param style. conf.py enables sphinx.ext.napoleon, which converts
# Google-style "Args:" / "Returns:" / "Raises:" to :param / :returns / :raises
# before or when autodoc-process-docstring runs. So sometimes :param exists for
# bindings, not raw "Args:". Both are supported.
# Match ":param name: desc" (no type) so we can insert type: ":param type name: desc"
_PARAM_LINE_RE = re.compile(r"^(\s*):param\s+(\w+):\s*(.*)$")
# Match ":param type name: desc" (already has type) - we skip or could replace
_PARAM_TYPED_LINE_RE = re.compile(r"^(\s*):param\s+(?:[\w.]+\s+)(\w+):\s*(.*)$")

# Section headers we treat as "Args" for type injection (case-insensitive)
_ARGS_SECTION_HEADERS = ("args:", "arguments:", "parameters:")


def _normalize_docstring_lines(lines: List[str]) -> None:
    """
    Expand in place any element that contains newlines into multiple lines.
    Some runtimes (e.g. nanobind/pybind) may pass the docstring as one or few
    strings with embedded \\n, so we never see a line that equals 'Args:'.
    """
    i = 0
    while i < len(lines):
        line = lines[i]
        if "\n" in line:
            parts = line.split("\n")
            lines[i:i + 1] = parts
            i += len(parts)
        else:
            i += 1


def _find_all_args_sections(lines: List[str]) -> List[Tuple[int, int]]:
    """Return [(start, end), ...] for every Args/Arguments/Parameters block (e.g. overloaded docstrings)."""
    regions: List[Tuple[int, int]] = []
    i = 0
    while i < len(lines):
        line_at_i = lines[i] if i < len(lines) else ""
        stripped = line_at_i.strip().lower()
        if stripped in _ARGS_SECTION_HEADERS:
            start = i
            end = start + 1
            while end < len(lines):
                next_line = lines[end]
                if next_line.strip() == "":
                    end += 1
                    continue
                if next_line.startswith(" ") or next_line.startswith("\t"):
                    end += 1
                    continue
                break
            regions.append((start, end))
            i = end
            continue
        i += 1
    return regions


def enrich_docstring_args_with_types(
    lines: List[str], param_types: Dict[str, str]
) -> None:
    """
    Modify docstring lines in place: for each Args line that has a parameter
    but no type in parentheses, insert the type from param_types (from the
    function signature).
    """
    if not param_types:
        return
    _normalize_docstring_lines(lines)
    regions = _find_all_args_sections(lines)
    if regions:
        for start, end in regions:
            for i in range(start + 1, end):
                line = lines[i]
                m = _ARGS_LINE_RE.match(line)
                if not m:
                    continue
                indent, param_name, existing_type, rest = m.groups()
                if existing_type and existing_type.strip():
                    continue
                type_str = param_types.get(param_name)
                if not type_str:
                    continue
                lines[i] = f"{indent}{param_name} ({type_str}): {rest}"
        return
    # Napoleon converts Args to :param; support that form too
    for i, line in enumerate(lines):
        m = _PARAM_LINE_RE.match(line)
        if not m or _PARAM_TYPED_LINE_RE.match(line):
            continue
        indent, param_name, rest = m.groups()
        type_str = param_types.get(param_name)
        if type_str:
            lines[i] = f"{indent}:param {type_str} {param_name}: {rest}"


def process_docstring_for_autodoc(
    app,
    what: str,
    name: str,
    obj,
    options,
    lines: List[str],
) -> None:
    """
    Sphinx autodoc-process-docstring handler: only for C++/nanobind bindings
    (ouster.sdk._bindings.* from python/src/cpp), inject parameter types from
    the function signature into the Args section when the docstring omits them.

    The event is provided by sphinx.ext.autodoc, not by core Sphinx.
    """
    if obj is None or what not in ("function", "method") or not _is_binding(obj):
        return
    param_types = get_signature_param_types(obj)
    enrich_docstring_args_with_types(lines, param_types)
