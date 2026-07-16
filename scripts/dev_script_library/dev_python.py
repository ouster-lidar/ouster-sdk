import click
import sys
import os
import glob
import platform
import subprocess
import warnings


def _macos_platform_from_arch(macos_wheel_arch):
    if macos_wheel_arch == "arm64":
        return "macosx-14.0-arm64", "-arch arm64"
    if macos_wheel_arch == "x86_64":
        return "macosx-14.0-x86_64", "-arch x86_64"
    raise ValueError(f"Unsupported macOS wheel arch: {macos_wheel_arch}")


def _infer_macos_wheel_arch():
    machine = platform.machine().lower()
    if machine in ("arm64", "aarch64"):
        return "arm64"
    if machine in ("x86_64", "amd64", "x86_64h"):
        return "x86_64"
    return None


def _apply_macos_wheel_env(env):
    """Set wheel packaging env vars from platform.machine() before pip wheel."""
    wheel_arch = _infer_macos_wheel_arch()
    if wheel_arch is None:
        machine = platform.machine()
        warnings.warn(
            "Cannot infer macOS wheel architecture from platform.machine()="
            f"{machine!r}. Without _PYTHON_HOST_PLATFORM and ARCHFLAGS, pip may "
            "tag universal2 wheels causing whl override in CI generated artifacts.",
            UserWarning,
            stacklevel=2,
        )
        return

    python_host_platform, archflags = _macos_platform_from_arch(wheel_arch)
    env["_PYTHON_HOST_PLATFORM"] = python_host_platform
    env["ARCHFLAGS"] = archflags
    print(
        "[MACOS WHEEL TAGGING] "
        f"_PYTHON_HOST_PLATFORM={python_host_platform} "
        f"ARCHFLAGS={archflags}"
    )


def _run_sdk_prebuild(ctx, env, build_type, extra_cmake_args):
    """Configure, build, and install the shared library set (with perception absorbed).

    On Windows, ouster_ceres_deps.dll is also built alongside shared_library.dll
    to avoid LNK1189 (>65 535 .lib members) when Ceres and its transitive static
    deps would otherwise be absorbed in full into a single DLL.

    Returns the install prefix path, which should be set as OUSTER_SDK_PREBUILT_DIR
    for the subsequent pip build so _bindings only needs to compile itself.
    """
    cmake_exe = ctx.obj.build_options.cmake_bin
    sdk_dir = ctx.obj.sdk_dir
    prebuild_build = os.path.join(ctx.obj.python_build_dir, "sdk_prebuilt_build")
    prebuild_install = os.path.join(ctx.obj.python_build_dir, "sdk_prebuilt_install")
    os.makedirs(prebuild_build, exist_ok=True)

    # Ensure the library_cleanups's Python dependencies are available.
    ctx.obj.build_libs.check_for_python_libs([
        "tree_sitter",
        "tree_sitter_cpp",
        ("clang.cindex", "libclang"),
    ])

    configure_cmd = [
        cmake_exe, sdk_dir,
        "-GNinja",
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_INSTALL_PREFIX={prebuild_install}",
        f"-DPYTHON_EXECUTABLE={sys.executable}",
        "-DBUILD_TESTING=OFF",
        "-DBUILD_EXAMPLES=OFF",
        "-DBUILD_PYTHON_MODULE=OFF",
        "-DBUILD_SHARED_LIBRARY=ON",
        "-DBUILD_PERCEPTION=ON",
    ]
    if ctx.obj.build_options.vcpkg_toolchain:
        configure_cmd.append(
            f"-DCMAKE_TOOLCHAIN_FILE={ctx.obj.build_options.vcpkg_toolchain}")
    if ctx.obj.build_options.vcpkg_triplet:
        configure_cmd.append(
            f"-DVCPKG_TARGET_TRIPLET={ctx.obj.build_options.vcpkg_triplet}")
    configure_cmd += list(extra_cmake_args)

    jobs = ctx.obj.build_options.threads
    print(f"\n[PREBUILD] Configuring: {' '.join(configure_cmd)}")
    subprocess.run(configure_cmd, cwd=prebuild_build, env=env, check=True)
    print(f"\n[PREBUILD] Building ({jobs} jobs)…")
    subprocess.run(
        [cmake_exe, "--build", ".", "--parallel", str(jobs), "--config", build_type],
        cwd=prebuild_build, env=env, check=True)
    print(f"\n[PREBUILD] Installing to {prebuild_install}…")
    subprocess.run(
        [cmake_exe, "--install", ".", "--config", build_type],
        cwd=prebuild_build, env=env, check=True)

    print(f"\n[PREBUILD] Done — shared library installed to {prebuild_install}")
    return prebuild_install


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
@click.option('--no-clean', default=False,
              is_flag=True,
              help='Do not clean build directories after building.')
@click.option('--cmake-arg', 'extra_cmake_args', multiple=True,
              help='Additional arguments forwarded to cmake configure step. '
                   'Use multiple --cmake-arg entries for separate tokens.')
@click.option('--verbose', is_flag=True, default=False,
              help='Enable verbose output from pip.')
@click.option('--prebuild-shared-lib', is_flag=True, default=False,
              help='Pre-build shared_library (including perception) once before '
                   'the pip build. Avoids repeated Eigen/CGAL/OSF compilation. '
                   'On Windows also builds ouster_ceres_deps.dll to avoid LNK1189.')
def python_build(ctx, output_type, vcpkg_toolchain,
                 vcpkg_triplet, threads, use_system_libs, build_type,
                 no_manifest_mode, profile_build, cmake_log_file,
                 coverage_flags, no_clean, extra_cmake_args, verbose,
                 prebuild_shared_lib):
    """Build Python SDK."""

    manifest_mode = not no_manifest_mode
    ctx.obj.build_options.process_args(
        vcpkg_toolchain=vcpkg_toolchain,
        vcpkg_triplet=vcpkg_triplet, threads=threads,
        manifest_mode=manifest_mode, use_system_libs=use_system_libs,
        coverage_flags=coverage_flags)
    ctx.obj.build_libs.check_for_python_libs(["nanobind", "wheel"])
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)
    ctx.obj.build_libs.check_native_build_tools()

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
    _vcpkg_keys = [
        "VCPKG_MANIFEST_MODE", "VCPKG_TARGET_TRIPLET", "VCPKG_TOOLCHAIN_FILE",
        "VCPKG_MAX_CONCURRENCY", "VCPKG_ROOT", "VCPKG_MANIFEST_DIR",
    ]
    if not ctx.obj.build_options.use_system_libs:
        opts = ctx.obj.build_options
        env.update({
            "VCPKG_MANIFEST_MODE": "ON" if opts.manifest_mode else "OFF",
            "VCPKG_TARGET_TRIPLET": opts.vcpkg_triplet,
            "VCPKG_TOOLCHAIN_FILE": opts.vcpkg_toolchain,
            "VCPKG_MAX_CONCURRENCY": str(opts.threads),
            "VCPKG_ROOT": ctx.obj.vcpkg_dir,
            "VCPKG_MANIFEST_DIR": ctx.obj.sdk_dir,
        })
    else:
        for key in _vcpkg_keys:
            env.pop(key, None)
    if "OUSTER_SDK_CMAKE_ARGS" not in env:
        env["OUSTER_SDK_CMAKE_ARGS"] = ""
    env["OUSTER_SDK_CMAKE_ARGS"] += " ".join(extra_cmake_args)
    print(f"Using CMake arguments: {env['OUSTER_SDK_CMAKE_ARGS']}")

    env["OUSTER_SDK_BUILD_JOBS"] = str(ctx.obj.build_options.threads)
    env["OUSTER_SDK_PATH"] = ctx.obj.sdk_dir

    # Ensure pip is new enough to support PEP 517 builds (pip 19.x lacks it).
    import subprocess
    subprocess.call([sys.executable, "-m", "pip", "install", "-q",
                     "--upgrade", "pip"])

    if prebuild_shared_lib:
        prebuilt_install = _run_sdk_prebuild(ctx, env, build_type, extra_cmake_args)
        env["OUSTER_SDK_PREBUILT_DIR"] = prebuilt_install
        print(f"[PREBUILD] Per-build will use pre-built SDK at {prebuilt_install}")

    python_dir = os.path.join(ctx.obj.sdk_dir, "python")
    args = [sys.executable, "-m", "pip"]
    if output_type == "editable":
        args.append("install")
        if no_clean:
            args.append("--no-clean")
        args.append(python_dir)
    elif output_type == "wheel":
        args.append("wheel")
        if no_clean:
            args.append("--no-clean")
        args.extend(["--no-deps", "--wheel-dir", ctx.obj.sdk_artifact_dir,
                     python_dir])
        if platform.system() == "Darwin":
            _apply_macos_wheel_env(env)
    if verbose:
        args.append("--verbose")
    cap_file = None
    profiling_started = False
    try:
        if profile_build:
            cap_file = os.path.join(ctx.obj.python_build_dir, "capture")
            run.run_command("ClangBuildAnalyzer", "--start",
                            ctx.obj.python_build_dir)
            profiling_started = True
        run.run_command(*args, cwd=python_dir, env=env)
        if output_type == "wheel":
            ctx.obj.print_output_location("Python SDK Wheel")
        elif output_type == "editable":
            print("Python SDK built in editable mode in the current environment")
    except Exception:
        print("Error building Python SDK")
        if profile_build and profiling_started:
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
@click.option('--skip-integration-tests',
              is_flag=True,
              default=False,
              help='Skip running integration tests.')
@click.option('--skip-perception-tests',
              is_flag=True,
              default=False,
              help='Skip tests marked @pytest.mark.perception (for builds without distributed binaries).')
def python_test(ctx, test_data_dir, threads, skip_integration_tests, skip_perception_tests):
    """Run Python tests."""
    ctx.obj.build_libs.check_for_python_libs([("xdist", "pytest-xdist"), ("pytest_asyncio", "pytest-asyncio")])
    ctx.obj.build_options.process_args(threads=threads)
    if test_data_dir is None and ctx.obj.internal_test_data_dir is not None:
        if ctx.obj.internal_test_data_tag is not None:
            test_data_dir = os.path.join(ctx.obj.internal_test_data_dir,
                                         ctx.obj.internal_test_data_tag)

    run = ctx.obj.build_libs.RunCommand(tty=True)
    python_test_dir = os.path.join(ctx.obj.sdk_dir, "python", "tests")
    args = [sys.executable, "-m", "pytest", "-n",
            str(ctx.obj.build_options.threads)]
    if skip_perception_tests:
        args += ["-m", "not perception"]
    run.run_command(*args, cwd=python_test_dir)
    if test_data_dir is None and "TEST_DATA_DIR" in os.environ:
        test_data_dir = os.environ["TEST_DATA_DIR"]
    if test_data_dir is None or skip_integration_tests:
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
