"""
Helpers that provide multi-language API links tied to the synced tab
selection (Python / C++).
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

from docutils import nodes
from docutils.utils import unescape
from sphinx import addnodes
from sphinx.domains import Domain
from sphinx.roles import SphinxRole
from sphinx.util import logging
from sphinx.util.nodes import split_explicit_title

logger = logging.getLogger(__name__)


class TabbedAPILink(nodes.inline):
    """Inline node that carries multi-language metadata."""


class TabbedClassRole(SphinxRole):
    """
    Role that captures a single reference target (``.SensorInfo``) and
    defers resolving it for each language variant until the doctree is
    fully resolved.
    """

    default_lang = "py"
    lang_specs: Dict[str, Tuple[str, str]] = {
        "py": ("py", "class"),
        "cpp": ("cpp", "class"),
    }

    def run(self) -> Tuple[Iterable[nodes.Node], Iterable[nodes.system_message]]:
        has_explicit, title, target_spec = split_explicit_title(self.text)
        title = unescape(title).strip()
        target_spec = unescape(target_spec).strip()

        if not target_spec:
            target_spec = title
        if not target_spec:
            return [], []

        lang_targets = self._parse_lang_targets(target_spec)
        if not lang_targets:
            return [], []

        displayed = title
        if not has_explicit:
            displayed = lang_targets.get(self.default_lang, next(iter(lang_targets.values())))

        default_target = lang_targets.get(self.default_lang)
        if default_target and default_target.startswith("~"):
            stripped = default_target[1:]
            if not has_explicit:
                displayed = stripped.split(".")[-1]
            lang_targets[self.default_lang] = stripped

        logger.debug(
            "[TabbedClassRole] rawtext=%r targets=%r displayed=%r explicit=%s",
            self.rawtext,
            lang_targets,
            displayed,
            has_explicit,
        )

        node = TabbedAPILink(self.rawtext, displayed, classes=list(self.options.get("class", [])))
        node["ouster_tabbed_pending"] = True
        node["ouster_targets"] = lang_targets
        node["ouster_title"] = displayed
        node["ouster_has_explicit"] = has_explicit
        node["ouster_lang_specs"] = dict(self.lang_specs)
        node["ouster_default_lang"] = self.default_lang
        node["ouster_ref_context"] = dict(self.env.ref_context)
        node["ouster_rawtext"] = self.rawtext
        node["ouster_sync_group"] = "api-lang"

        return [node], []

    def _parse_lang_targets(self, raw: str) -> Dict[str, str]:
        parts = [piece.strip() for piece in raw.split("|") if piece.strip()]
        lang_targets: Dict[str, str] = {}
        fallback: Optional[str] = None
        for part in parts:
            if "=" in part:
                prefix, value = part.split("=", 1)
                prefix = prefix.strip()
                value = value.strip()
                if prefix in self.lang_specs and value:
                    lang_targets[prefix] = value
                    continue
            if fallback is None:
                fallback = part

        if not lang_targets and fallback:
            for lang in self.lang_specs:
                lang_targets[lang] = fallback
        else:
            for lang in self.lang_specs:
                if lang not in lang_targets and fallback:
                    lang_targets[lang] = fallback

        return lang_targets


class TabbedFunctionRole(TabbedClassRole):
    """Tabbed role for multi-language function references."""

    lang_specs = {
        "py": ("py", "func"),
        "cpp": ("cpp", "func"),
    }


class TabbedEnumRole(TabbedClassRole):
    """Tabbed role for enums (Python class stub, C++ enum)."""

    lang_specs = {
        "py": ("py", "class"),
        "cpp": ("cpp", "enum"),
    }


class OusterTabbedDomain(Domain):
    """
    Lightweight domain to expose ``:ouster:class:``.
    """

    name = "ouster"
    label = "Ouster multi-language helpers"
    roles = {
        "class": TabbedClassRole(),
        "func": TabbedFunctionRole(),
        "enum": TabbedEnumRole(),
    }
    directives: Dict[str, object] = {}
    object_types: Dict[str, object] = {}
    indices: Tuple = ()

    def resolve_xref(self, env, fromdocname, builder, typ, target, node, contnode):
        # These nodes never reach Sphinx's standard xref resolver because
        # we convert them ourselves in the doctree-resolved step.
        return None


def _resolve_target_uri(app, fromdocname, node, lang: str, spec: Tuple[str, str]) -> Optional[str]:
    env = app.builder.env
    domain_name, ref_type = spec
    domain = env.domains.get(domain_name)
    if not domain:
        logger.info("Unknown domain '%s' for ouster tabbed link", domain_name)
        return None

    lang_targets = node.get("ouster_targets", {})
    target = lang_targets.get(lang)
    if not target:
        return None

    pending = addnodes.pending_xref(
        node.get("ouster_rawtext", ""),
        refdomain=domain_name,
        reftype=ref_type,
        refexplicit=node["ouster_has_explicit"],
    )
    pending["reftarget"] = target
    for key, value in node.get("ouster_ref_context", {}).items():
        pending[key] = value

    contnode = nodes.inline(node.get("ouster_rawtext", ""), node["ouster_title"])
    resolver_args = (env, fromdocname, app.builder, ref_type, target, pending, contnode)
    try:
        resolved = domain.resolve_xref(*resolver_args)
    except TypeError:
        # Older versions of Sphinx omit the ``env`` argument.
        resolved = domain.resolve_xref(
            fromdocname, app.builder, ref_type, target, pending, contnode
        )

    if not resolved:
        # Fallback: try to find group functions in Exhale-generated RST files
        if domain_name == "cpp" and ref_type == "func":
            fallback_uri = _try_resolve_group_function(app, target, fromdocname)
            if fallback_uri:
                return fallback_uri
        logger.warning(f"Unable to resolve ouster tabbed link for lang {lang},target {target}")
        return None

    if isinstance(resolved, nodes.reference):
        if "refuri" in resolved:
            return resolved["refuri"]
        if "refid" in resolved:
            return f"#{resolved['refid']}"

    if isinstance(resolved, nodes.Element):
        refuri = resolved.get("refuri")
        if refuri:
            return refuri
        refid = resolved.get("refid")
        if refid:
            return f"#{refid}"

    return None


def _try_resolve_group_function(app, target: str, fromdocname: str) -> Optional[str]:
    """
    Try to resolve a C++ function reference that might be in a Doxygen group.
    Looks for Exhale-generated RST files for group functions.
    """
    # Extract function name from target like "ouster::sdk::namespace::xxx"
    target = target.strip()
    match = re.match(r'(?:.*::\s*)?(\S+)(?:\s*\(.*)?$', target)
    if not match:
        return None

    function_name = match.group(1)
    if not function_name:
        return None

    # Look for Exhale-generated group function RST files
    confdir = Path(app.confdir)
    cpp_api_dir = confdir / "cpp" / "api_cpp"
    if not cpp_api_dir.exists():
        logger.debug(f"[_try_resolve_group_function] C++ API dir not found: {cpp_api_dir}")
        return None

    # Search for RST files that might contain this function
    pattern = f"function_group__*{function_name}*.rst"
    matching_files = list(cpp_api_dir.glob(pattern))

    if not matching_files:
        return None
    rst_file = matching_files[0]
    # Convert RST path to docname (relative to confdir, without .rst extension)
    docname = str(rst_file.relative_to(confdir).with_suffix(""))

    # Return reference to this document
    uri = app.builder.get_relative_uri(fromdocname, docname)
    logger.info(f"[_try_resolve_group_function] Resolved group function '{target}' -> '{docname}' -> '{uri}'")
    return uri


def process_tabbed_api_links(app, doctree, fromdocname):
    if (
        fromdocname.startswith("cpp/api_cpp/")
        or fromdocname.startswith("python/api_python/")
    ):
        return

    tab_nodes = list(doctree.findall(TabbedAPILink))

    for tab_node in tab_nodes:
        lang_map = tab_node.get("ouster_lang_specs", {})
        targets = {}
        for lang, spec in lang_map.items():
            uri = _resolve_target_uri(app, fromdocname, tab_node, lang, spec)
            if uri:
                targets[lang] = uri
            else:
                logger.warning(
                    "[process_tabbed_api_links] unresolved target for lang=%s spec=%s",
                    lang,
                    spec,
                )

        if not targets:
            logger.warning("[process_tabbed_api_links] no targets resolved")
            # Nothing resolved; drop down to plain text so the build still works.
            tab_node.replace_self(nodes.inline("", tab_node["ouster_title"]))
            continue

        replacement = nodes.reference(tab_node["ouster_rawtext"], "", internal=True)
        replacement += nodes.Text(tab_node["ouster_title"])
        replacement["classes"].extend(tab_node.get("classes", []))
        replacement["classes"].append("ouster-tabbed-api-link")

        sync_group = tab_node.get("ouster_sync_group", "api-lang")
        replacement["ouster_sync_group"] = sync_group

        default_lang = tab_node.get("ouster_default_lang", "py")
        default_uri = targets.get(default_lang)
        if not default_uri:
            default_uri = next(iter(targets.values()))
        replacement["refuri"] = default_uri
        replacement["ouster_default_lang"] = default_lang
        replacement["ouster_tabbed_targets"] = targets

        try:
            tab_node.replace_self(replacement)
        except AttributeError as exc:
            logger.error(
                "[process_tabbed_api_links] failed to replace node=%r (type=%s) with %r: %s",
                tab_node,
                type(tab_node),
                replacement,
                exc,
            )
            raise


def _patch_html_reference(app):
    from sphinx.writers.html5 import HTML5Translator

    if getattr(HTML5Translator, "_ouster_tabbed_patched", False):
        return

    original_visit = HTML5Translator.visit_reference
    original_depart = HTML5Translator.depart_reference

    def visit_reference(self, node):
        targets = node.get("ouster_tabbed_targets")
        if not targets:
            return original_visit(self, node)

        atts = {"class": "reference"}
        if node.get("internal") or "refuri" not in node:
            atts["class"] += " internal"
        else:
            atts["class"] += " external"
        if "refuri" in node:
            atts["href"] = node["refuri"] or "#"
            if (
                self.settings.cloak_email_addresses
                and atts["href"].startswith("mailto:")
            ):
                atts["href"] = self.cloak_mailto(atts["href"])
                self.in_mailto = True
        else:
            assert "refid" in node, 'References must have "refuri" or "refid" attribute.'
            atts["href"] = f"#{node['refid']}"
        if not isinstance(node.parent, nodes.TextElement):
            assert len(node) == 1 and isinstance(node[0], nodes.image)
            atts["class"] += " image-reference"
        if "reftitle" in node:
            atts["title"] = node["reftitle"]
        if "target" in node:
            atts["target"] = node["target"]
        if "rel" in node:
            atts["rel"] = node["rel"]

        sync_group = node.get("ouster_sync_group")
        if sync_group:
            atts["data-ouster-sync-group"] = sync_group
        default_lang = node.get("ouster_default_lang")
        if default_lang:
            atts["data-ouster-default-lang"] = default_lang
        for lang, uri in targets.items():
            atts[f"data-ouster-target-{lang}"] = uri

        self.body.append(self.starttag(node, "a", "", **atts))
        if node.get("secnumber"):
            numbers = ".".join(map(str, node["secnumber"]))
            self.body.append(f"{numbers}{self.secnumber_suffix}")

    def depart_reference(self, node):
        if node.get("ouster_tabbed_targets"):
            self.body.append("</a>")
            if not isinstance(node.parent, nodes.TextElement):
                self.body.append("\n")
            self.in_mailto = False
        else:
            original_depart(self, node)

    HTML5Translator.visit_reference = visit_reference
    HTML5Translator.depart_reference = depart_reference
    HTML5Translator._ouster_tabbed_patched = True


def setup(app):
    app.add_config_value("ouster_no_cpp_api", False, "env")
    app.add_config_value("ouster_no_python_api", False, "env")
    app.add_domain(OusterTabbedDomain)

    if app.config.ouster_no_cpp_api and app.config.ouster_no_python_api:
        return

    app.connect("builder-inited", _patch_html_reference)
    app.connect("doctree-resolved", process_tabbed_api_links)
