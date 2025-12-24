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
        run_apidoc(self.app)
        RstProcessor(self.app).process_all()

    def cleanup(self, exception=None):
        cleanup_doxygen(self.app, exception)


def setup(app):
    """
    Sphinx auto api extension entry point.
    """
    orchestrator = AutoApiDocsGenerator(app)

    def on_config_inited(_app, _config):
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
