from util import configure_logger
from apidoc import run_apidoc
from doxygen_runner import run_doxygen, cleanup_doxygen
from rst_processor import RstProcessor


class AutoApiDocsGenerator:
    """
      1. Run Doxygen build to generate cpp docs
      2. Run sphinx-apidoc for python API docs
      3. RST processing (add pybind, single read write for now)
    """
    def __init__(self, app):
        self.app = app
        self.log = configure_logger("ouster.docs.autoapidocsgenerator")

    def build_sphinx(self):
        if self.app.config.ouster_no_python_api:
            self.log.info("Skipping apidoc and pybind processing (ouster_no_python_api)")
            return
        self.log.info("Running sphinx-apidoc for Python API RST generation")
        run_apidoc(self.app)
        self.log.info("Running RST processor (pybind sections, merge/sort)")
        RstProcessor(self.app).process_all()
        self.log.info("Python API RST generation complete")

    def cleanup(self, exception=None):
        if not self.app.config.ouster_no_cpp_api:
            cleanup_doxygen(self.app, exception)


def setup(app):
    """
    Sphinx auto api extension entry point.
    """
    orchestrator = AutoApiDocsGenerator(app)

    def on_config_inited(_app, _config):
        if _app.config.ouster_no_cpp_api:
            return
        run_doxygen(_app)

    def on_builder_inited(_app):
        orchestrator.build_sphinx()

    def on_build_finished(_app, exception):
        orchestrator.cleanup(exception)

    app.connect("config-inited", on_config_inited)
    app.connect("builder-inited", on_builder_inited)
    app.connect("build-finished", on_build_finished)

    return {
        "parallel_read_safe": True,
        "parallel_write_safe": True
    }
