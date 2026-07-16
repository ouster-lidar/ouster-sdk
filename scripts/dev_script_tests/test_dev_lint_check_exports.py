import sys
import os
import pytest
from unittest.mock import MagicMock, patch
from click.testing import CliRunner

mock_clang = MagicMock()


class MockCursorKind:
    NAMESPACE = "NAMESPACE"
    CLASS_DECL = "CLASS_DECL"
    STRUCT_DECL = "STRUCT_DECL"
    CLASS_TEMPLATE = "CLASS_TEMPLATE"
    FUNCTION_DECL = "FUNCTION_DECL"
    CXX_METHOD = "CXX_METHOD"
    CONSTRUCTOR = "CONSTRUCTOR"
    DESTRUCTOR = "DESTRUCTOR"
    ANNOTATE_ATTR = "ANNOTATE_ATTR"
    TEMPLATE_TYPE_PARAMETER = "TEMPLATE_TYPE_PARAMETER"
    TEMPLATE_NON_TYPE_PARAMETER = "TEMPLATE_NON_TYPE_PARAMETER"
    FUNCTION_TEMPLATE = "FUNCTION_TEMPLATE"
    PARM_DECL = "PARM_DECL"


class MockAccessSpecifier:
    PUBLIC = "PUBLIC"
    PRIVATE = "PRIVATE"
    PROTECTED = "PROTECTED"


mock_clang.cindex.CursorKind = MockCursorKind
mock_clang.cindex.AccessSpecifier = MockAccessSpecifier
mock_clang.cindex.conf.lib.__dict__ = {"clang_CXXMethod_isDeleted": True}
patcher = patch.dict(sys.modules, {"clang": mock_clang, "clang.cindex": mock_clang.cindex})
patcher.start()

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))


import dev_lint_check_exports as target_module  # noqa E402


class MockCursor:
    def __init__(self, kind, spelling="", children=None, parent=None,
                 access=MockAccessSpecifier.PUBLIC, is_deleted=False,
                 filename="test.h"):
        self.kind = kind
        self.spelling = spelling
        self.displayname = spelling
        self._children = children if children else []
        self.lexical_parent = parent
        self.access_specifier = access
        self._is_deleted = is_deleted

        self.location = MagicMock()
        self.location.file.name = filename
        self.location.file = filename
        self.location.line = 10
        self.location.column = 5

        for child in self._children:
            child.lexical_parent = self

    def get_children(self):
        return self._children

    def is_deleted_method(self):
        return self._is_deleted

    def add_child(self, child):
        child.lexical_parent = self
        self._children.append(child)
        return self


def create_annotation(text):
    return MockCursor(MockCursorKind.ANNOTATE_ATTR, spelling=text)


class TestProcessAst:
    """Tests the core logic of AST traversal and validation."""

    def test_correct_annotation(self):
        annotation = create_annotation("OUSTER_API_CLASS")
        node_class = MockCursor(MockCursorKind.CLASS_DECL, "MyClass", children=[annotation])
        node_ns = MockCursor(MockCursorKind.NAMESPACE, "ouster", children=[node_class])
        root = MockCursor(MockCursorKind.NAMESPACE, "root", children=[node_ns])

        wrong, correct, missing = target_module.process_ast(root)

        assert len(wrong) == 0
        assert len(missing) == 0
        assert len(correct) == 1
        assert "MyClass" in correct[0]

    def test_missing_annotation(self):
        # Add dummy to class so it's not a forward decl
        dummy_child = MockCursor(MockCursorKind.PARM_DECL, "dummy")
        node_class = MockCursor(MockCursorKind.CLASS_DECL, "MyClass", children=[dummy_child])

        node_ns = MockCursor(MockCursorKind.NAMESPACE, "ouster", children=[node_class])
        root = MockCursor(MockCursorKind.NAMESPACE, "root", children=[node_ns])

        wrong, correct, missing = target_module.process_ast(root)

        assert len(missing) == 1
        assert "MyClass" in missing[0]
        assert "Expected: OUSTER_API_CLASS" in missing[0]

    def test_wrong_annotation(self):
        annotation = create_annotation("OUSTER_API_FUNCTION")
        node_class = MockCursor(MockCursorKind.CLASS_DECL, "MyClass", children=[annotation])
        node_ns = MockCursor(MockCursorKind.NAMESPACE, "ouster", children=[node_class])
        root = MockCursor(MockCursorKind.NAMESPACE, "root", children=[node_ns])

        wrong, correct, missing = target_module.process_ast(root)

        assert len(wrong) == 1
        assert "MyClass" in wrong[0]
        assert "Expected: OUSTER_API_CLASS" in wrong[0]

    def test_ignore_impl_namespace(self):
        node_class = MockCursor(MockCursorKind.CLASS_DECL, "HiddenClass", children=[])
        node_impl = MockCursor(MockCursorKind.NAMESPACE, "impl", children=[node_class])
        node_ouster = MockCursor(MockCursorKind.NAMESPACE, "ouster", children=[node_impl])
        root = MockCursor(MockCursorKind.NAMESPACE, "root", children=[node_ouster])

        wrong, correct, missing = target_module.process_ast(root)

        assert len(missing) == 0
        assert len(wrong) == 0

    def test_ignore_private_members(self):
        annotation_class = create_annotation("OUSTER_API_CLASS")
        node_method = MockCursor(MockCursorKind.CXX_METHOD, "secret", access=MockAccessSpecifier.PRIVATE)
        node_class = MockCursor(MockCursorKind.CLASS_DECL, "MyClass", children=[annotation_class, node_method])
        node_ouster = MockCursor(MockCursorKind.NAMESPACE, "ouster", children=[node_class])

        wrong, correct, missing = target_module.process_ast(node_ouster)

        assert len(correct) == 1
        assert len(missing) == 0

    def test_ignore_deleted_members(self):
        node_method = MockCursor(MockCursorKind.CXX_METHOD, "no_op", is_deleted=True)
        # Add dummy to class so it's not a forward decl
        dummy = MockCursor(MockCursorKind.PARM_DECL, "d")
        node_class = MockCursor(MockCursorKind.CLASS_DECL, "MyClass", children=[node_method, dummy])
        node_ouster = MockCursor(MockCursorKind.NAMESPACE, "ouster", children=[node_class])

        wrong, correct, missing = target_module.process_ast(node_ouster)

        assert len(missing) == 1
        assert "MyClass" in missing[0]
        assert "no_op" not in missing[0]

    def test_ignore_templates(self):
        node_tmpl = MockCursor(MockCursorKind.CLASS_TEMPLATE, "TmplClass")
        param = MockCursor(MockCursorKind.TEMPLATE_TYPE_PARAMETER, "T")
        node_tmpl.add_child(param)
        node_ouster = MockCursor(MockCursorKind.NAMESPACE, "ouster", children=[node_tmpl])

        wrong, correct, missing = target_module.process_ast(node_ouster)

        assert len(missing) == 0


class TestCheckCppExportsCommand:
    @pytest.fixture
    def mock_pool(self):
        """
        Mock the Pool object used as a context manager (with Pool(...) as pool:).
        Pool.__new__ returns a context manager whose __enter__ returns the pool instance.
        """
        pool_instance = MagicMock()

        def side_effect(func, args):
            res = MagicMock()
            res.get.return_value = ([], [], args[0])
            res.ready.return_value = True
            return res

        pool_instance.apply_async.side_effect = side_effect
        # Support use as a context manager
        pool_instance.__enter__ = MagicMock(return_value=pool_instance)
        pool_instance.__exit__ = MagicMock(return_value=False)
        return pool_instance

    def test_command_basic_execution(self, mock_context, mock_pool, tmp_path):
        runner = CliRunner()

        with patch(f"{target_module.__name__}.process") as mock_process, \
             patch(f"{target_module.__name__}.Pool", return_value=mock_pool):

            mock_process.return_value = ([], [], "test_file.h")

            (tmp_path / "sdk/ouster_core/include/ouster/core").mkdir(parents=True, exist_ok=True)
            (tmp_path / "sdk/ouster_core/include/ouster/core/test.h").touch()

            result = runner.invoke(target_module.check_cpp_exports, obj=mock_context)

            if result.exit_code != 0:
                print(result.output)

            assert result.exit_code == 0
            assert "No issues detected" in result.output

    def test_command_with_errors(self, mock_context, mock_pool):
        """Test that the command fails if issues are returned."""
        runner = CliRunner()

        # Define a side_effect specific to this test that returns ERRORS
        def error_side_effect(func, args):
            res = MagicMock()
            # Return tuple: (missing, wrong, filename)
            res.get.return_value = (["Missing Annotation X"], ["Wrong Annotation Y"], "bad_file.h")
            res.ready.return_value = True
            return res

        # Apply the error side_effect to the mock pool instance
        mock_pool.apply_async.side_effect = error_side_effect

        mock_context.build_options.run_vcpkg_initialized_check = MagicMock()
        with patch("os.path.isdir", return_value=False), \
             patch(f"{target_module.__name__}.process"), \
             patch(f"{target_module.__name__}.Pool", return_value=mock_pool):

            # We explicitly pass a file path to trigger the logic
            result = runner.invoke(target_module.check_cpp_exports, ["bad_file.h"], obj=mock_context)

            assert result.exit_code != 0  # Should now fail
            assert "Issues found in C++ SDK exports" in result.output
            assert "Missing Annotation X" in result.output

    def test_process_function_integration(self):
        mock_index = MagicMock()
        mock_tu = MagicMock()
        mock_cursor = MockCursor(MockCursorKind.NAMESPACE, "ouster")
        mock_tu.cursor = mock_cursor
        mock_index.parse.return_value = mock_tu

        mock_clang.cindex.Index.create.return_value = mock_index

        mock_cmds = MagicMock()
        mock_cmd = MagicMock()
        mock_cmd.arguments = ["-I/some/path", "-c", "input.cpp"]
        mock_cmds.getCompileCommands.return_value = [mock_cmd]

        mock_clang.cindex.CompilationDatabase.fromDirectory.return_value = mock_cmds

        missing, wrong, fname = target_module.process("my_file.h", "/tmp")

        mock_clang.cindex.CompilationDatabase.fromDirectory.assert_called_with("/tmp")
        mock_cmds.getCompileCommands.assert_called_with("my_file.h")

        call_args = mock_index.parse.call_args
        assert call_args[0][0] == "my_file.h"
        assert "-DOUSTER_CHECK_EXPORTS=1" in call_args[0][1]
        assert "-I/some/path" in call_args[0][1]

        assert len(missing) == 0
        assert len(wrong) == 0
