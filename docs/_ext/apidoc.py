import subprocess
from pathlib import Path
from util import configure_logger


def run_apidoc(app):
    """
    Equivalent to original _run_sphinx_apidoc implementation.
    """
    log = configure_logger("ouster.docs.apidoc")
    confdir = Path(app.confdir)
    python_src = confdir.parent / "python" / "src" / "ouster"
    output_dir = confdir / "python" / "api_python"

    if not python_src.exists():
        log.error("Python source path missing: %s", python_src)
        return

    # Clean previous generated RST
    if output_dir.exists():
        for f in output_dir.glob("*.rst"):
            f.unlink()
    output_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        "sphinx-apidoc",
        "-o", str(output_dir),
        str(python_src),
        str(python_src / "cli"),
        "--force",
        "--separate",
        "--implicit-namespaces",
    ]

    templ_dir = confdir / "_templates" / "apidoc"
    if templ_dir.exists():
        cmd.extend(["--templatedir", str(templ_dir)])

    log.info("Running sphinx-apidoc")
    subprocess.check_call(cmd)