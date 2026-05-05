import click
import sys
import os
import glob


@click.command(name="python")
@click.pass_context
@click.option('--output-type',
              type=click.Choice(['editable', 'wheel'],
                                case_sensitive=False),
              default='editable')
@click.option('--vcpkg-toolchain',
              default=None,
              help='Path to alternative vcpkg toolchain file.')
@click.option('--vcpkg-triplet',
              default=None,
              help='Vcpkg triplet to use with the alternative toolchain file.')
@click.option('--use-system-libs',
              default=False,
              is_flag=True,
              help='Use system libraries instead of vcpkg.')
@click.option('--threads',
              default=None,
              help='Number of threads to use.')
@click.option('--build-type', default="Release",
              type=click.Choice(['Release', 'Debug', 'RelWithDebInfo'],
                                case_sensitive=False))
@click.option('--no-manifest-mode',
              default=False,
              is_flag=True,
              help='Disable vcpkg manifest mode.')
@click.option('--profile-build', default=False,
              is_flag=True,
              help='Enable build profiling with CMake and clang.')
@click.option('--cmake-log-file',
              default=None,
              help='Path to CMake log file when profiling is enabled.')
@click.option('--coverage-flags', default=False,
              is_flag=True,
              help='Enable coverage flags for the build.')
def python_build(ctx, output_type, vcpkg_toolchain,
                 vcpkg_triplet, threads, use_system_libs, build_type,
                 no_manifest_mode, profile_build, cmake_log_file,
                 coverage_flags):
    """Build Python SDK."""

    manifest_mode = not no_manifest_mode
    ctx.obj.build_options.process_args(
        vcpkg_toolchain=vcpkg_toolchain,
        vcpkg_triplet=vcpkg_triplet, threads=threads,
        manifest_mode=manifest_mode, use_system_libs=use_system_libs,
        coverage_flags=coverage_flags)
    ctx.obj.build_libs.check_for_python_lib("pybind11")
    ctx.obj.build_libs.check_for_python_lib("wheel")
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)

    run = ctx.obj.build_libs.RunCommand(tty=True)

    if profile_build:
        ctx.obj.build_libs.check_for_tool("clang")
        ctx.obj.build_libs.check_for_tool("ClangBuildAnalyzer")

    ctx.obj.build_options.run_vcpkg_initialized_check()

    env = ctx.obj.build_options.build_env
    if cmake_log_file is not None:
        env["OUSTER_SDK_CMAKE_LOG_FILE"] = cmake_log_file
    if profile_build:
        env["CMAKE_CXX_COMPILER"] = "clang++"
        env["CMAKE_C_COMPILER"] = "clang"
        env["CMAKE_CXX_FLAGS"] = "-ftime-trace"
        env["CMAKE_C_FLAGS"] = "-ftime-trace"
        env["OUSTER_BUILD_DIR_COPY"] = ctx.obj.python_build_dir
        env["USE_OPENMP"] = "OFF"
    env["BUILD_TYPE"] = build_type
    if not ctx.obj.build_options.use_system_libs:
        env["VCPKG_MANIFEST_MODE"] = "ON" if ctx.obj.build_options.manifest_mode else "OFF"
        env["VCPKG_TARGET_TRIPLET"] = ctx.obj.build_options.vcpkg_triplet
        env["VCPKG_TOOLCHAIN_FILE"] = ctx.obj.build_options.vcpkg_toolchain
        env["VCPKG_MAX_CONCURRENCY"] = str(ctx.obj.build_options.threads)
        env["VCPKG_ROOT"] = ctx.obj.vcpkg_dir
        env["VCPKG_MANIFEST_DIR"] = ctx.obj.sdk_dir
    env["OUSTER_SDK_BUILD_JOBS"] = str(ctx.obj.build_options.threads)

    python_dir = os.path.join(ctx.obj.sdk_dir, "python")
    args = []
    if output_type == "editable":
        args = [sys.executable, "-m", "pip", "install",
                python_dir]
    elif output_type == "wheel":
        args = [sys.executable, "-m", "pip", "wheel",
                "--no-deps", "--wheel-dir", ctx.obj.sdk_artifact_dir,
                python_dir]

    cap_file = None
    try:
        if profile_build:
            cap_file = os.path.join(ctx.obj.python_build_dir, "capture")
            run.run_command("ClangBuildAnalyzer", "--start",
                            ctx.obj.python_build_dir)
        run.run_command(*args, cwd=python_dir, env=env)
        if output_type == "wheel":
            ctx.obj.print_output_location("Python SDK Wheel")
        elif output_type == "editable":
            print("Python SDK built in editable mode in the current environment")
    except Exception:
        print("Error building Python SDK")
        if profile_build:
            cap_file = os.path.join(ctx.obj.python_build_dir, "capture")
            run.run_command("ClangBuildAnalyzer", "--stop",
                           ctx.obj.python_build_dir, cap_file)
        raise RuntimeError("Please check the output for details.")

    if profile_build:
        run.run_command("ClangBuildAnalyzer", "--stop",
                        ctx.obj.python_build_dir, cap_file)
        run.run_command("ClangBuildAnalyzer", "--analyze",
                        cap_file)
        perf_jsons = glob.glob(os.path.join(ctx.obj.python_build_dir, "**", "*.dir", "**", "*.json"), recursive=True)
        combined_file = os.path.join(ctx.obj.python_build_dir, "combined_perf.json")
        ctx.obj.build_libs.perf_json_combine(perf_jsons, combined_file)
        print(f"Combined profile data at: {combined_file}")
        print("You can view the profile data using Chrome tracing: about:tracing")


@click.command(name="python")
@click.pass_context
@click.option('--test-data-dir',
              default=None,
              help='Directory for the test data.')
@click.option('--threads',
              default=None,
              help='Number of threads to use.')
def python_test(ctx, test_data_dir, threads):
    """Run Python tests."""
    ctx.obj.build_libs.check_for_python_lib("xdist", display_name="pytest-xdist")
    ctx.obj.build_options.process_args(threads=threads)
    if test_data_dir is not None:
        pass
    elif ctx.obj.internal_test_data_dir is not None:
        if ctx.obj.internal_test_data_tag is not None:
            test_data_dir = os.path.join(ctx.obj.internal_test_data_dir,
                                         ctx.obj.internal_test_data_tag)

    run = ctx.obj.build_libs.RunCommand(tty=True)
    python_test_dir = os.path.join(ctx.obj.sdk_dir, "python", "tests")
    args = [sys.executable, "-m", "pytest", '-n',
            str(ctx.obj.build_options.threads)]
    run.run_command(*args, cwd=python_test_dir)
    if test_data_dir is None and "TEST_DATA_DIR" in os.environ:
        test_data_dir = os.environ["TEST_DATA_DIR"]
    if test_data_dir is None:
        print("Missing, test data dir, skipping internal tests")
    else:
        env = os.environ.copy()
        env["TEST_DATA_DIR"] = test_data_dir
        python_integration_test_dir = os.path.join(ctx.obj.sdk_dir,
                                                   "tests", "integration")
        args = [sys.executable, "-m", "pytest",
                '-n', ctx.obj.build_options.threads]
        try:
            run.run_command(*args, cwd=python_integration_test_dir, env=env)
        except Exception:
            print("Error running Python integration tests")
            raise RuntimeError("Please check the output for details.")


def import_module(click_context):
    click_context.build_group.add_command(python_build)
    click_context.test_group.add_command(python_test)


def finalize(click_context):
    pass
