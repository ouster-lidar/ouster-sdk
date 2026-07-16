"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.

Deprecation utilities for Python bindings.

This module provides the building blocks used throughout the SDK to keep old,
deprecated public API surfaces working while steering users toward their
replacements. It covers two distinct kinds of deprecation:

* **Renamed keyword arguments.** As part of the scan -> frame terminology
  migration many functions, methods and constructors had keyword arguments
  renamed (for example ``scan`` -> ``frame``). The ``*_deprecated_kwargs``
  helpers and :func:`resolve_deprecated_kwargs` accept the old names, emit a
  :class:`FutureWarning`, and forward the value under the new name so existing
  call sites keep working.

* **Renamed objects.** Classes, functions and other module-level objects that
  were renamed are kept reachable under their old name via
  :func:`deprecated_alias`, which installs a warning-emitting shim into a
  module namespace.

The companion module :mod:`ouster.sdk._kwarg_aliases` uses these helpers to
wire up every renamed parameter across the SDK's binding and pure-Python
modules.

Warnings are emitted with :class:`FutureWarning` (rather than the
:class:`DeprecationWarning` Python hides by default) so that they are visible
to end users by default, and ``stacklevel`` is chosen so the warning points at
the user's call site rather than at this module.
"""

import functools
import inspect
import warnings
from typing import Any, Callable, Dict, Generic, Mapping, Optional, Sequence, Type, TypeVar, Union, overload


T = TypeVar('T')  # Generic type for return values

#: Default ``last_supported_version`` for the scan -> frame migration. This is
#: the last SDK release in which the deprecated keyword names below are promised
#: to keep working; it is surfaced in the warning message so users know their
#: deadline for migrating.
LAST_SUPPORTED_SCAN_FRAME_VERSION = "1.0"

#: Canonical mapping of every deprecated keyword name to its replacement,
#: produced by the scan -> frame terminology migration. Keys are the old
#: (deprecated) names and values are the new names. This single table is reused
#: by :mod:`ouster.sdk._kwarg_aliases` so that every patched function, method
#: and constructor accepts the same consistent set of legacy names. Note that
#: an individual call site usually only installs the subset of these aliases
#: that are relevant to its own signature.
SCAN_FRAME_KWARG_ALIASES: Dict[str, str] = {
    "scan": "frame",
    "scans": "frames",
    "source_scan": "source_frame",
    "target_scan": "target_frame",
    "lidar_scan": "lidar_frame",
    "lidar_scan_encoder": "lidar_frame_encoder",
    "scan_num": "frame_num",
    "scan_source": "frame_set_source",
    "scan_iter": "frame_set_iter",
    "scans_iter": "frames_iter",
    "last_n_scans": "last_n_frame_sets",
    "new_scans": "new_frames",
    "scan_ts": "frame_ts",
    "scan_duration": "frame_duration",
    "prev_scan_time": "prev_frame_time",
    "ls": "lf",
    "_override_lidarscanviz": "_override_lidar_frame_viz",
}


def warn_deprecated(message: str,
                    category: Type[Warning] = FutureWarning,
                    *,
                    stacklevel: int = 2) -> None:
    """Emit a deprecation warning attributed to the caller's frame.

    Thin wrapper around :func:`warnings.warn` used as the single choke point for
    all deprecation warnings in the SDK. Centralising it makes the warning
    category consistent and easy to change in one place.

    Args:
        message: The human-readable warning text to display.
        category: The warning class to raise. Defaults to :class:`FutureWarning`
            (rather than :class:`DeprecationWarning`) so the warning is shown to
            end users by default instead of being filtered out.
        stacklevel: How many stack frames up to attribute the warning to. The
            default of ``2`` blames this function's immediate caller; helpers in
            this module pass a larger value so the warning points past their own
            wrapper frames and lands on the user's call site.
    """
    warnings.warn(message, category, stacklevel=stacklevel)


def _get_kwarg_deprecation_message(old_name: str,
                                   new_name: str,
                                   last_supported_version: str) -> str:
    """Build the standard warning text for a deprecated keyword argument.

    Args:
        old_name: The deprecated keyword name the caller used.
        new_name: The replacement keyword name they should switch to.
        last_supported_version: The last SDK version in which ``old_name`` will
            be accepted.

    Returns:
        A formatted, single-sentence warning message naming both the old and
        new keyword and the removal version.
    """
    return (f"Keyword argument '{old_name}' is deprecated: "
            f"use '{new_name}' instead. "
            f"The last supported version for this will be {last_supported_version}.")


def resolve_deprecated_kwargs(
    kwargs: Dict[str, Any],
    aliases: Mapping[str, str],
    *,
    last_supported_version: str = LAST_SUPPORTED_SCAN_FRAME_VERSION,
    context: str = "",
) -> Dict[str, Any]:
    """Return a copy of ``kwargs`` with deprecated keyword names remapped.

    This is the core routine shared by every keyword-deprecation helper in this
    module. For each alias whose old name is present in ``kwargs`` it emits a
    :class:`FutureWarning` and moves the value to the new name. Keys that do not
    match any alias are passed through untouched.

    Args:
        kwargs: The keyword arguments supplied by the caller. Not mutated; a
            shallow copy is returned.
        aliases: Mapping of deprecated name -> replacement name to apply. Only
            entries whose key appears in ``kwargs`` have any effect.
        last_supported_version: Version string embedded in the warning message
            to tell the user when the old name will stop working.
        context: Optional prefix (typically a function or method qualified name)
            prepended to the warning so the user can tell which call site
            triggered it.

    Returns:
        A new dict equivalent to ``kwargs`` but with every deprecated key
        replaced by its modern counterpart.

    Raises:
        TypeError: If the caller passes both a deprecated name and its
            replacement (for example both ``scan`` and ``frame``), since the
            intended value would be ambiguous.
    """
    resolved = dict(kwargs)
    for old_name, new_name in aliases.items():
        if old_name not in resolved:
            continue
        if new_name in kwargs:
            raise TypeError(
                f"Got both deprecated keyword argument '{old_name}' and "
                f"replacement '{new_name}'"
            )
        message = _get_kwarg_deprecation_message(
            old_name, new_name, last_supported_version)
        if context:
            message = f"{context}: {message}"
        warn_deprecated(message, stacklevel=3)
        resolved[new_name] = resolved.pop(old_name)
    return resolved


@overload
def deprecated_kwargs(_func: Callable[..., T], /) -> Callable[..., T]:
    ...


@overload
def deprecated_kwargs(
    *,
    last_supported_version: str = LAST_SUPPORTED_SCAN_FRAME_VERSION,
    **aliases: str,
) -> Callable[[Callable[..., T]], Callable[..., T]]:
    ...


def deprecated_kwargs(  # type: ignore[misc]
    _func: Optional[Callable[..., Any]] = None,
    *,
    last_supported_version: str = LAST_SUPPORTED_SCAN_FRAME_VERSION,
    **aliases: str,
) -> Union[Callable[..., Any], Callable[[Callable[..., T]], Callable[..., T]]]:
    """Decorator that lets a function accept deprecated keyword argument names.

    Supports both bare and parameterised forms::

        @deprecated_kwargs
        def f(frame): ...

        @deprecated_kwargs(scan="frame", last_supported_version="1.0")
        def g(frame): ...

    When the decorated function is called, any deprecated keyword in
    ``aliases`` is remapped to its replacement (with a warning) before the call
    is forwarded. Positional arguments are preserved: the wrapper binds the
    incoming arguments against the function's real signature, remaps only the
    keyword portion, and re-invokes the original function. This makes it safe to
    decorate pure-Python callables whose signatures are introspectable.

    For nanobind bindings, whose signatures are not always introspectable or
    which require positional-only passing, use
    :func:`wrap_deprecated_kwargs` or
    :func:`wrap_nb_positional_deprecated_kwargs` instead.

    Args:
        _func: The function being decorated when used bare (``@deprecated_kwargs``).
            Left as ``None`` when used with arguments. Passed positionally only.
        last_supported_version: Version reported in the warning message.
        **aliases: Deprecated-name -> new-name pairs to accept, supplied as
            keyword arguments (for example ``scan="frame"``).

    Returns:
        The wrapped function when called bare, or a decorator that wraps a
        function when called with keyword arguments.
    """

    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        sig = inspect.signature(func)

        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            bound = sig.bind_partial(*args, **kwargs)
            bound.apply_defaults()
            remapped = resolve_deprecated_kwargs(
                dict(bound.kwargs),
                aliases,
                last_supported_version=last_supported_version,
                context=func.__qualname__,
            )
            positional = bound.args
            return func(*positional, **remapped)

        return wrapper  # type: ignore[return-value]

    if _func is not None:
        return decorator(_func)
    return decorator


def patch_init_deprecated_kwargs(
    cls: type,
    *,
    last_supported_version: str = LAST_SUPPORTED_SCAN_FRAME_VERSION,
    **aliases: str,
) -> None:
    """Patch ``cls.__init__`` in place to accept deprecated keyword names.

    Replaces the class's ``__init__`` with a wrapper that remaps deprecated
    keywords (emitting a warning) before delegating to the original
    constructor. Use this for pure-Python classes and for nanobind classes that
    still run user code through ``__init__``.

    Note:
        This does *not* work for nanobind classes that construct in their
        ``tp_new`` slot, where reassigning ``__init__`` is a silent no-op. Such
        classes need a dedicated constructor overload in the C++ binding
        instead (see the ``FrameSet`` note in
        :mod:`ouster.sdk._kwarg_aliases`).

    Args:
        cls: The class whose ``__init__`` should be patched. Modified in place.
        last_supported_version: Version reported in the warning message.
        **aliases: Deprecated-name -> new-name pairs to accept on the
            constructor.
    """
    original_init = cls.__init__  # type: ignore[misc]

    @functools.wraps(original_init)
    def patched_init(self, *args: Any, **kwargs: Any) -> None:
        kwargs = resolve_deprecated_kwargs(
            kwargs,
            aliases,
            last_supported_version=last_supported_version,
            context=f"{cls.__name__}.__init__",
        )
        original_init(self, *args, **kwargs)

    cls.__init__ = patched_init  # type: ignore[method-assign, misc]


def patch_method_deprecated_kwargs(
    cls: type,
    method_name: str,
    *,
    last_supported_version: str = LAST_SUPPORTED_SCAN_FRAME_VERSION,
    **aliases: str,
) -> None:
    """Patch a method on ``cls`` in place to accept deprecated keyword names.

    Looks up ``method_name`` on the class, wraps it so deprecated keywords are
    remapped (with a warning) before the call, and reassigns the wrapped
    version back onto the class. Works for both pure-Python and nanobind
    methods.

    Args:
        cls: The class owning the method to patch. Modified in place.
        method_name: Name of the method to wrap (for example ``"update"`` or
            ``"__call__"``).
        last_supported_version: Version reported in the warning message.
        **aliases: Deprecated-name -> new-name pairs to accept on the method.
    """
    original = getattr(cls, method_name)

    @functools.wraps(original)
    def patched(*args: Any, **kwargs: Any) -> Any:
        kwargs = resolve_deprecated_kwargs(
            kwargs,
            aliases,
            last_supported_version=last_supported_version,
            context=f"{cls.__name__}.{method_name}",
        )
        return original(*args, **kwargs)

    setattr(cls, method_name, patched)


def wrap_nb_positional_deprecated_kwargs(
    func: Callable[..., T],
    arg_names: Sequence[str],
    *,
    last_supported_version: str = LAST_SUPPORTED_SCAN_FRAME_VERSION,
    **aliases: str,
) -> Callable[..., T]:
    """Wrap a nanobind function that requires positional argument passing.

    Some nanobind free functions do not accept keyword arguments at all and
    must be called purely positionally. This wrapper lets callers keep using
    keyword arguments (including the deprecated names) by collecting them,
    remapping the deprecated ones, and then reassembling a positional call in
    the order given by ``arg_names``.

    The wrapper has two modes:

    * If any positional arguments are supplied, the call is forwarded as-is
      (deprecated keywords still remapped) and trusted to be well-formed.
    * If the call is keyword-only, every name in ``arg_names`` must be present
      after remapping; the values are then passed positionally in
      ``arg_names`` order.

    Args:
        func: The nanobind function to wrap.
        arg_names: The function's positional parameters, in order. Used both to
            validate that a keyword-only call is complete and to determine the
            order in which to pass the collected values positionally.
        last_supported_version: Version reported in the warning message.
        **aliases: Deprecated-name -> new-name pairs to accept.

    Returns:
        A wrapper around ``func`` that accepts the deprecated keyword names.

    Raises:
        TypeError: If the call is keyword-only and one or more names in
            ``arg_names`` are missing after remapping.
    """
    func_name = getattr(func, "__name__", repr(func))

    @functools.wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        kwargs = resolve_deprecated_kwargs(
            kwargs,
            aliases,
            last_supported_version=last_supported_version,
            context=func_name,
        )
        if args:
            return func(*args, **kwargs)
        missing = [name for name in arg_names if name not in kwargs]
        if missing:
            raise TypeError(
                f"{func_name}() missing required keyword arguments: "
                f"{', '.join(repr(name) for name in missing)}"
            )
        return func(*[kwargs[name] for name in arg_names])

    return wrapper  # type: ignore[return-value]


def wrap_deprecated_kwargs(
    func: Callable[..., T],
    *,
    last_supported_version: str = LAST_SUPPORTED_SCAN_FRAME_VERSION,
    **aliases: str,
) -> Callable[..., T]:
    """Return ``func`` wrapped to accept deprecated keyword argument names.

    The general-purpose, signature-agnostic counterpart to
    :func:`deprecated_kwargs`. Unlike the decorator it does not inspect or bind
    the wrapped callable's signature; it simply remaps the deprecated keywords
    (with a warning) and forwards ``*args`` and ``**kwargs`` straight through.
    This makes it suitable for nanobind functions whose signatures cannot be
    introspected, as long as they accept keyword arguments.

    Args:
        func: The callable to wrap.
        last_supported_version: Version reported in the warning message.
        **aliases: Deprecated-name -> new-name pairs to accept.

    Returns:
        A wrapper around ``func`` that accepts the deprecated keyword names and
        otherwise behaves identically.
    """
    func_name = getattr(func, "__name__", repr(func))

    @functools.wraps(func)
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        kwargs = resolve_deprecated_kwargs(
            kwargs,
            aliases,
            last_supported_version=last_supported_version,
            context=func_name,
        )
        return func(*args, **kwargs)

    return wrapper  # type: ignore[return-value]


def _get_deprecation_message(old_name: str, new_name: str, last_supported_version: str) -> str:
    """Generate a standard deprecation warning message.

    Args:
        old_name: The deprecated name
        new_name: The new name to use instead
        last_supported_version: The version after which this alias will be removed

    Returns:
        A formatted deprecation warning message
    """
    return (f"{old_name} is deprecated: Use {new_name} instead. "
            f"The last supported version for this will be {last_supported_version}.")


def deprecated_alias(old_name: str, new_name: str, obj: object,
                  module_dict: Dict[str, object],
                  last_supported_version: str) -> None:
    """
    Create a deprecated alias for an object with appropriate warning.

    Installs ``old_name`` into ``module_dict`` so that the renamed object stays
    importable under its previous name while warning users to migrate. The
    strategy depends on whether ``obj`` is callable:

    * **Callable** (classes, functions): a wrapper function is installed that
      warns and then calls through to ``obj``. The warning fires on every call.
      Note that because the alias is a function, ``isinstance`` checks and
      subclassing against the deprecated *class* name will not behave like the
      real class; use the new name for those.
    * **Non-callable** (constants, instances): a ``DeprecatedAttribute`` proxy
      is installed that forwards attribute access, comparisons and common
      conversions to the wrapped value and warns once, the first time it is
      touched.

    Typically called at module scope as, e.g.,
    ``deprecated_alias("LidarScan", "LidarFrame", LidarFrame, globals(), "1.0")``.

    Args:
        old_name: The deprecated name to expose.
        new_name: The new name to use instead (named in the warning).
        obj: The object being aliased (the value reachable under ``new_name``).
        module_dict: The module's ``__dict__`` (usually ``globals()``) to add
            the alias to.
        last_supported_version: The version after which this alias will be removed.
    """
    version = last_supported_version

    def _deprecated_wrapper(*args, **kwargs):
        warn_deprecated(
            _get_deprecation_message(old_name, new_name, version),
            stacklevel=2,
        )
        if callable(obj):
            return obj(*args, **kwargs)
        return obj

    # For non-callable objects, we need a different approach
    if not callable(obj):
        class DeprecatedAttribute(Generic[T]):
            def __init__(self, value: T, old_name: str, new_name: str, version: str):
                self._value = value
                self._old_name = old_name
                self._new_name = new_name
                self._version = version
                self._warned = False

            def _warn_if_needed(self):
                if not self._warned:
                    warn_deprecated(
                        _get_deprecation_message(self._old_name, self._new_name, self._version),
                        stacklevel=3,
                    )
                    self._warned = True

            def __getattr__(self, name):
                self._warn_if_needed()
                return getattr(self._value, name)

            def __repr__(self):
                self._warn_if_needed()
                return repr(self._value)

            def __str__(self):
                self._warn_if_needed()
                return str(self._value)

            def __eq__(self, other):
                self._warn_if_needed()
                return self._value == other

            def __hash__(self):
                self._warn_if_needed()
                return hash(self._value)

            def __ne__(self, other):
                self._warn_if_needed()
                return self._value != other

            def __bool__(self):
                self._warn_if_needed()
                return bool(self._value)

            def __int__(self):
                self._warn_if_needed()
                return int(self._value)

            def __float__(self):
                self._warn_if_needed()
                return float(self._value)

        module_dict[old_name] = DeprecatedAttribute(obj, old_name, new_name, version)
    else:
        module_dict[old_name] = _deprecated_wrapper
