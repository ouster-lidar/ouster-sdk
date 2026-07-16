"""Tests for the po_example snippet."""
from __future__ import annotations

import os
import runpy
import warnings
from pathlib import Path

import pytest

CURRENT_DIR = Path(__file__).resolve().parent
REPO_ROOT = CURRENT_DIR.parents[4]
SCRIPT_PATH = CURRENT_DIR / "po_example.py"

def test_po_example_runs_and_generates_files(tmp_path):
    """Test that pose optimizer example runs without error and generates output files."""
    if os.environ.get("ENABLE_SLOW_DOC_TESTS", "0") != "1":
        pytest.skip("ENABLE_SLOW_DOC_TESTS not set")

    _env_root = os.environ.get("TEST_DATA_DIR")
    if not _env_root:
        warnings.warn("TEST_DATA_DIR not set", RuntimeWarning)
        pytest.skip("TEST_DATA_DIR not set")
    ROOT_DIR = Path(_env_root)
    OSF_PATH = ROOT_DIR / "mapping" / "loop.osf"

    # Verify the main script exists
    assert SCRIPT_PATH.exists(), "po_example.py is missing"
    
    # Verify OSF fixture exists
    assert OSF_PATH.exists(), f"Required test fixture not found: {OSF_PATH}"
    
    # Define output files in tmp_path
    output_osf = tmp_path / "po_output.osf"
    trajectory_csv = tmp_path / "loop_test_traj.csv"
    
    # Set environment variables for the script
    original_osf_env = os.environ.get("POSE_OPTIMIZER_OSF")
    original_output_env = os.environ.get("POSE_OPTIMIZER_OUTPUT")
    original_cwd = os.getcwd()
    
    try:
        # Change to tmp_path so trajectory file is created there
        os.chdir(tmp_path)
        os.environ["POSE_OPTIMIZER_OSF"] = str(OSF_PATH)
        os.environ["POSE_OPTIMIZER_OUTPUT"] = str(output_osf)
        
        # Run the pose optimizer example and check it doesn't throw
        runpy.run_path(str(SCRIPT_PATH), run_name="__main__")
        
        # Check that the trajectory file was created
        assert trajectory_csv.exists(), f"Expected trajectory file '{trajectory_csv.name}' was not created"
        
        # Check that the OSF output file was created
        assert output_osf.exists(), f"Expected OSF output file '{output_osf.name}' was not created"
        
    except Exception as e:
        pytest.fail(f"Pose optimizer example threw an exception: {e}")
    finally:
        # Restore original state
        os.chdir(original_cwd)
        if original_osf_env is not None:
            os.environ["POSE_OPTIMIZER_OSF"] = original_osf_env
        elif "POSE_OPTIMIZER_OSF" in os.environ:
            del os.environ["POSE_OPTIMIZER_OSF"]
        if original_output_env is not None:
            os.environ["POSE_OPTIMIZER_OUTPUT"] = original_output_env
        elif "POSE_OPTIMIZER_OUTPUT" in os.environ:
            del os.environ["POSE_OPTIMIZER_OUTPUT"]
