import subprocess
import tempfile
from pathlib import Path
from string import Template
from util import configure_logger


def run_doxygen(app):
    if not getattr(app.config, "breathe_projects", None):
        return

    log = configure_logger("ouster.docs.doxygen")
    log.info("Running Doxygen")

    confdir = Path(app.confdir)
    template_path = confdir / "Doxyfile"
    if not template_path.exists():
        log.warning("Doxyfile template not found: %s", template_path)
        return

    tmpdir = Path(tempfile.mkdtemp())
    doxyfile = tmpdir / "Doxyfile"
    app.config._temp_doxy_dir = str(tmpdir)
    substitutions = {
        'project': app.config.project,
        'version': app.config.version,
        'output_dir': app.doctreedir,
        'enhanced_warnings': ("YES" if getattr(app.config, 'enhanced_warnings', False) else "NO"),
        'warn_log_file': str(Path(app.doctreedir) / "warning_log.log"),
    }

    template_content = template_path.read_text()
    doxyfile.write_text(Template(template_content).substitute(substitutions))
    # Using check_call as it displays output while run on console
    # CalledProcessError is raised on failure
    subprocess.check_call(["doxygen", str(doxyfile)], cwd=str(confdir))

    # Update breathe project paths
    for name, rel in app.config.breathe_projects.items():
        app.config.breathe_projects[name] = str(Path(app.doctreedir) / rel)


def cleanup_doxygen(app, exception=None):
    """
    Remove temp directory if Doxygen was run.
    """
    temp_dir = getattr(app.config, '_temp_doxy_dir', None)
    if not temp_dir:
        return
    p = Path(temp_dir)
    if p.exists():
        import shutil
        shutil.rmtree(p, ignore_errors=True)