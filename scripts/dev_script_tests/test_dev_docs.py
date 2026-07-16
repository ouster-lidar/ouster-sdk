import os
import pytest
import sys
from unittest.mock import MagicMock, patch
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

import dev_docs  # noqa E402


@pytest.fixture
def mock_context_docs(mock_context):
    """Mocks the Click context object (ctx.obj)."""
    ctx_obj = mock_context

    # Mock the internal Doxygen class wrapper
    mock_doxy_instance = MagicMock()
    mock_doxy_instance.generate_doxygen.return_value = None
    mock_doxy_instance.get_warnings.return_value = []

    # Ensure the Doxygen class constructor returns our mock instance
    ctx_obj.build_libs.Doxygen.return_value = mock_doxy_instance
    ctx_obj.build_libs.parse_version.return_value = "1.0.0"

    return ctx_obj


def create_dummy_xml(content, filepath):
    """Helper to write XML content to a file."""
    with open(filepath, 'w') as f:
        f.write(content)


def test_extended_doc_check_valid(tmp_path):
    """Test that valid [in]/[out] tags produce no errors."""
    xml_content = """
    <doxygen>
        <memberdef kind="function">
            <type>void</type>
            <name>testFunc</name>
            <argsstring>(int a)</argsstring>
            <location file="test.cpp" line="10" column="1"/>
            <param>
                <declname>a</declname>
                <briefdescription>
                    <para>[in] This is a valid input.</para>
                </briefdescription>
            </param>
            <parameterlist kind="param">
                <parameteritem>
                    <parameternamelist>
                        <parametername direction="in">a</parametername>
                    </parameternamelist>
                </parameteritem>
            </parameterlist>
        </memberdef>
    </doxygen>
    """
    f = tmp_path / "valid.xml"
    create_dummy_xml(xml_content, f)

    inline, block = dev_docs.extended_doc_check(str(f))
    assert len(inline) == 0
    assert len(block) == 0


def test_extended_doc_check_invalid_inline(tmp_path):
    """Test missing [in] tag in brief description."""
    xml_content = """
    <doxygen>
        <memberdef kind="function">
            <location file="test.cpp" line="10" column="1"/>
            <param>
                <declname>badParam</declname>
                <briefdescription>
                    <para>Missing direction tag here.</para>
                </briefdescription>
            </param>
        </memberdef>
    </doxygen>
    """
    f = tmp_path / "bad_inline.xml"
    create_dummy_xml(xml_content, f)

    inline, block = dev_docs.extended_doc_check(str(f))
    assert len(inline) == 1
    assert inline[0][1] == "badParam"


def test_extended_doc_check_invalid_block(tmp_path):
    """Test missing 'direction' attribute in parameter list."""
    xml_content = """
    <doxygen>
        <memberdef kind="function">
            <location file="test.cpp" line="10" column="1"/>
            <parameterlist kind="param">
                <parameteritem>
                    <parameternamelist>
                        <parametername>badBlockParam</parametername>
                    </parameternamelist>
                </parameteritem>
            </parameterlist>
        </memberdef>
    </doxygen>
    """
    f = tmp_path / "bad_block.xml"
    create_dummy_xml(xml_content, f)

    inline, block = dev_docs.extended_doc_check(str(f))
    assert len(block) == 1
    assert block[0][1] == "badBlockParam"


_skip_on_windows = pytest.mark.skipif(
    sys.platform == "win32",
    reason="build doxygen is not supported on Windows"
)


@_skip_on_windows
def test_doxygen_version_too_old(mock_context_docs):
    """Ensure RuntimeError if Doxygen version < 1.11.0."""
    runner = CliRunner()

    # Mock subprocess to return an old version
    with patch('subprocess.run') as mock_run:
        mock_run.return_value.stdout = b"1.8.17 (some hash)"

        result = runner.invoke(dev_docs.build_doxygen_docs, obj=mock_context_docs)

        assert result.exit_code != 0
        assert isinstance(result.exception, RuntimeError)
        assert "doxygen needs to be updated" in str(result.exception)


@_skip_on_windows
def test_doxygen_version_newer_warning(mock_context_docs):
    """Ensure Warning printed if Doxygen version > 1.11.0, but proceeds."""
    runner = CliRunner()

    with patch('subprocess.run') as mock_run, \
         patch('glob.glob', return_value=[]):

        mock_run.return_value.stdout = b"1.12.0 (some hash)"

        result = runner.invoke(dev_docs.build_doxygen_docs, obj=mock_context_docs)

        assert "WARNING: doxygen version is greater" in result.output
        assert result.exit_code == 0


@_skip_on_windows
def test_doxygen_warnings_filtering(mock_context_docs):
    """Test that specific warnings are ignored and others cause failure."""
    runner = CliRunner()

    # Setup the mock Doxygen instance to return warnings
    doxy_instance = mock_context_docs.build_libs.Doxygen.return_value
    doxy_instance.get_warnings.return_value = [
        "somefile.cpp:10: warning: standard doxygen warning",
        "macro.cpp:5: warning: doxygen could be confused by a macro call"
        "impl.cpp:9: warning: ouster::impl details"
    ]

    with patch('subprocess.run') as mock_run:
        mock_run.return_value.stdout = b"1.11.0"

        result = runner.invoke(dev_docs.build_doxygen_docs, obj=mock_context_docs)

        # Should fail because of the one standard warning
        assert result.exit_code != 0
        assert "Doxygen Warnings Exist: PLEASE FIX" in result.output

        # Verify filtering worked (count should be 1, not 3)
        assert "1 Doxygen Warnings" in result.output


@_skip_on_windows
def test_param_check_fail_integration(mock_context_docs):
    """Test that XML parsing issues cause the CLI to fail."""
    runner = CliRunner()

    with patch('subprocess.run') as mock_run, \
         patch('glob.glob', return_value=["dummy.xml"]), \
         patch('dev_docs.extended_doc_check') as mock_check:

        mock_run.return_value.stdout = b"1.11.0"

        # Mock extended_doc_check to return one issue
        mock_check.return_value = (
            [({'file': 'f.cpp', 'line': '1', 'function_def': 'void f()', 'column': '0'}, 'param1')],
            []
        )

        result = runner.invoke(dev_docs.build_doxygen_docs, obj=mock_context_docs)

        assert result.exit_code != 0
        assert "Parameter Direction Warnings Exist" in result.output
        assert "Parameter: param1" in result.output


@_skip_on_windows
def test_no_werror_flag(mock_context_docs):
    """Test that --no-werror prevents failure even with issues."""
    runner = CliRunner()

    # Setup warnings
    doxy_instance = mock_context_docs.build_libs.Doxygen.return_value
    doxy_instance.get_warnings.return_value = ["real warning"]

    with patch('subprocess.run') as mock_run, \
         patch('glob.glob', return_value=[]):

        mock_run.return_value.stdout = b"1.11.0"

        result = runner.invoke(dev_docs.build_doxygen_docs, ['--no-werror'], obj=mock_context_docs)

        assert "Doxygen Warnings" in result.output
        assert result.exit_code == 0


@_skip_on_windows
def test_log_file_generation(mock_context_docs, tmp_path):
    """Test that the parameter log is written to the specified path."""
    runner = CliRunner()
    log_file = tmp_path / "param_warnings.log"

    with patch('subprocess.run') as mock_run, \
         patch('glob.glob', return_value=["dummy.xml"]), \
         patch('dev_docs.extended_doc_check') as mock_check:

        mock_run.return_value.stdout = b"1.11.0"
        # Return an issue so we have something to log
        mock_check.return_value = ([({'file': 'a', 'line': '1',
                                      'function_def': 'a', 'column': '0'},
                                     'p')], [])

        # Run with --no-werror so we don't crash, just check the log
        result = runner.invoke(dev_docs.build_doxygen_docs,
                               ['--param-log', str(log_file), '--no-werror'],
                               obj=mock_context_docs)

        assert result.exit_code == 0
        assert log_file.exists()
        content = log_file.read_text()
        assert "Parameter: p" in content
