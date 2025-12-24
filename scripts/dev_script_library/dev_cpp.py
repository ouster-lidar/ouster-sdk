import click
import os
import glob


@click.command(name="cpp")
@click.pass_context
@click.option('--package', default=False,
              is_flag=True, help='Enable shared library build.')
@click.option('--cmake-bin',
              default="cmake",
              help='Pass in alternative cmakes binary.')
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
@click.option('--no-examples', default=False,
              is_flag=True, help='Disable examples build.')
@click.option('--hil-examples', default=False,
              is_flag=True, help='Disable HIL examples build.')
@click.option('--no-tests', default=False,
              is_flag=True, help='Disable tests build.')
@click.option('--install-dir', default=None,
              help='Install directory for C++ SDK. This also specifies where package artifacts go.')
@click.option('--cmake-arg', 'extra_cmake_args', multiple=True,
              help='Additional arguments forwarded to cmake configure step. '
                   'Use multiple --cmake-arg entries for separate tokens.')
@click.option('--profile-build', default=False,
              is_flag=True,
              help='Enable build profiling with CMake and clang.')
@click.option('--coverage-flags', default=False,
              is_flag=True,
              help='Enable coverage flags for the build.')
def cpp_build(ctx, package, cmake_bin, vcpkg_toolchain, vcpkg_triplet,
              use_system_libs, threads, build_type, no_manifest_mode,
              no_examples, hil_examples, no_tests, install_dir,
              extra_cmake_args, profile_build, coverage_flags):
    """Build C++ SDK."""
    manifest_mode = not no_manifest_mode
    ctx.obj.build_options.process_args(
        cmake_bin=cmake_bin, vcpkg_toolchain=vcpkg_toolchain,
        vcpkg_triplet=vcpkg_triplet, threads=threads,
        manifest_mode=manifest_mode, use_system_libs=use_system_libs,
        coverage_flags=coverage_flags)
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)
    ctx.obj.build_options.run_vcpkg_initialized_check()
    if profile_build:
        ctx.obj.build_libs.check_for_tool("clang")
        ctx.obj.build_libs.check_for_tool("ClangBuildAnalyzer")

    manifest_mode_text = "ON" if ctx.obj.build_options.manifest_mode else "OFF"
    testing_text = "OFF" if no_tests or package else "ON"
    examples_text = "OFF" if no_examples else "ON"
    hil_examples_text = "ON" if hil_examples else "OFF"
    cmake_args = [f"-DVCPKG_MANIFEST_MODE={manifest_mode_text}",
                  f"-DVCPKG_MAX_CONCURRENCY={ctx.obj.build_options.threads}",
                  f"-DBUILD_TESTING={testing_text}",
                  "-DBUILD_PCAP=ON",
                  "-DBUILD_OSF=ON",
                  "-DBUILD_SENSOR=ON",
                  "-DBUILD_MAPPING=ON",
                  f"-DBUILD_EXAMPLES={examples_text}",
                  f"-DRUN_CI_EXAMPLES={examples_text}",
                  f"-DRUN_HIL_EXAMPLES={hil_examples_text}"]
    if not no_tests and not package:
        cmake_args.append("-DOUSTER_EXTERNAL_TESTS=ON")
        cmake_args.append("-DOUSTER_INTERNAL_TESTS=ON")

    if profile_build:
        cmake_args.append("-DCMAKE_CXX_COMPILER=clang++")
        cmake_args.append("-DCMAKE_C_COMPILER=clang")
        cmake_args.append("-DCMAKE_CXX_FLAGS='-ftime-trace'")
        cmake_args.append("-DCMAKE_C_FLAGS='-ftime-trace'")
        cmake_args.append("-DUSE_OPENMP=OFF")
    if "VCPKG_BINARY_SOURCES" in ctx.obj.build_options.build_env:
        cmake_args.append("-DVCPKG_BINARY_SOURCES="
                          f"{ctx.obj.build_options.build_env['VCPKG_BINARY_SOURCES']}")

    if not ctx.obj.build_options.use_system_libs:
        cmake_args.extend([f"-DCMAKE_TOOLCHAIN_FILE={ctx.obj.build_options.vcpkg_toolchain}",
                           f"-DVCPKG_TARGET_TRIPLET={ctx.obj.build_options.vcpkg_triplet}"])

    if package:
        cmake_args.append("-DBUILD_SHARED_LIBRARY=ON")

    if extra_cmake_args:
        cmake_args.extend(list(extra_cmake_args))
    cmake = ctx.obj.build_libs.CMake(ctx.obj.sdk_dir,
                                     ctx.obj.cmake_build_dir,
                                     ctx.obj.sdk_artifact_dir,
                                     cmake_args=cmake_args,
                                     build_type=build_type,
                                     tty=True,
                                     env=ctx.obj.build_options.build_env,
                                     cmake_path=ctx.obj.build_options.cmake_bin)
    aux_run = ctx.obj.build_libs.RunCommand()
    cap_file = os.path.join(ctx.obj.cmake_build_dir, "capture")
    try:
        cmake.generate()
        if profile_build:
            aux_run.run_command("ClangBuildAnalyzer", "--start", ctx.obj.cmake_build_dir)
        cmake.build(threads=ctx.obj.build_options.threads)
        if profile_build:
            aux_run.run_command("ClangBuildAnalyzer", "--stop", ctx.obj.cmake_build_dir,
                                cap_file)

            perf_jsons = glob.glob(f"{ctx.obj.cmake_build_dir}/**/*.dir/*.json", recursive=True)
            combined_file = os.path.join(ctx.obj.cmake_build_dir, "combined_perf.json")
            ctx.obj.build_libs.perf_json_combine(perf_jsons, combined_file)
            print(f"Combined profile data at: {combined_file}")
            print("You can view the profile data using Chrome tracing: about:tracing")
        if install_dir:
            cmake.install(prefix=install_dir)
        if package:
            cmake.install(prefix=install_dir)
            cmake.make_package(prefix=install_dir)
            ctx.obj.print_output_location("C++ SDK Binary Package")
        else:
            ctx.obj.print_output_location("C++ SDK Build Dir")
    except Exception as e:
        print("Error building C++ SDK")
        raise RuntimeError(f"Please check the output for details: {e}")

    if package and not no_tests:
        test_lib_dir = os.path.join(ctx.obj.sdk_dir, "test_libs")
        # Make sure to install the built SDK for test linkage
        cmake.install(prefix=test_lib_dir)
        cmake_test_args = [f"-DVCPKG_MANIFEST_MODE={manifest_mode_text}",
                           f"-DVCPKG_MAX_CONCURRENCY={ctx.obj.build_options.threads}",
                           "-DOUSTER_SDK_TEST_SOURCE=DYNAMIC",
                           "-DOUSTER_INTERNAL_TESTS=OFF",
                           "-DOUSTER_EXTERNAL_TESTS=ON",
                           f"-DCMAKE_PREFIX_PATH={test_lib_dir}"]
        if not ctx.obj.build_options.use_system_libs:
            cmake_test_args.extend([f"-DCMAKE_TOOLCHAIN_FILE={ctx.obj.build_options.vcpkg_toolchain}",
                                    f"-DVCPKG_TARGET_TRIPLET={ctx.obj.build_options.vcpkg_triplet}"])
        if extra_cmake_args:
            cmake_test_args.extend(list(extra_cmake_args))
        os.makedirs(os.path.join(ctx.obj.cmake_build_dir, "shared_tests"), exist_ok=True)
        cmake_test = ctx.obj.build_libs.CMake(os.path.join(ctx.obj.sdk_dir, "tests"),
                                              os.path.join(ctx.obj.cmake_build_dir, "shared_tests"),
                                              ctx.obj.sdk_artifact_dir,
                                              cmake_args=cmake_test_args,
                                              build_type=build_type,
                                              tty=True,
                                              env=ctx.obj.build_options.build_env,
                                              cmake_path=ctx.obj.build_options.cmake_bin)
        cmake_test.generate()
        if profile_build:
            aux_run.run_command("ClangBuildAnalyzer", "--start", ctx.obj.cmake_build_dir)
        cmake_test.build(threads=ctx.obj.build_options.threads)
        if profile_build:
            aux_run.run_command("ClangBuildAnalyzer", "--stop", ctx.obj.cmake_build_dir,
                                cap_file)
    if profile_build:
        aux_run.run_command("ClangBuildAnalyzer", "--analyze", cap_file)


def find_latest_package(install_dir):
    package_files = glob.glob(os.path.join(install_dir, "ouster-sdk-*.zip"))
    if not package_files:
        raise FileNotFoundError("No C++ SDK package files found.")
    latest_file = max(package_files, key=os.path.getctime)
    return latest_file


@click.command(name="cpp")
@click.pass_context
@click.option('--threads',
              default=None,
              help='Number of threads to use.')
@click.option('--test-dir-override',
              default=None,
              type=click.Path(exists=True, file_okay=False, dir_okay=True,
                              resolve_path=True),
              help='Directory to run ctest from. Defaults to the CMake build dir.')
@click.option("--use-shared-libs", is_flag=True, default=False,
              help="Use shared library build for tests.")
@click.option('--shared-libs-path', default=None,
              type=click.Path(exists=True, file_okay=False, dir_okay=True,
                              resolve_path=True),
              help='Path to shared libraries to use for tests.')
@click.option('--ctest-bin', default="ctest", help='Path to ctest binary.')
@click.option('--build-type', default="Release",
              type=click.Choice(['Release', 'Debug']))
def cpp_test(ctx, threads, test_dir_override,
             use_shared_libs, shared_libs_path,
             ctest_bin, build_type):
    """Run C++ tests."""
    ctest_path = ctx.obj.build_libs.check_for_tool("ctest", tool_path=ctest_bin)
    ctx.obj.build_options.process_args(threads=threads)
    run = ctx.obj.build_libs.RunCommand(tty=True)

    test_cwd = None
    if not use_shared_libs:
        test_cwd = test_dir_override or ctx.obj.cmake_build_dir
    else:
        test_cwd = test_dir_override or os.path.join(ctx.obj.cmake_build_dir, "shared_tests")

    args = [ctest_path, "--output-on-failure", ".",
            "-j", str(ctx.obj.build_options.threads),
            "-C", build_type]
    try:
        run.run_command(*args, cwd=test_cwd)
    except Exception:
        print("Error running SDK tests.")
        raise RuntimeError("Please check the output for details.")


@click.command()
@click.pass_context
@click.option('--output', default=None,
              help='Output location.')
@click.option('--cmake-bin',
              default="cmake",
              help='Pass in alternative cmakes binary.')
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
@click.option('--no-manifest-mode',
              default=False,
              is_flag=True,
              help='Disable vcpkg manifest mode.')
@click.option('--threads',
              default=None,
              help='Number of threads to use.')
def compile_commands(ctx, output, cmake_bin, vcpkg_toolchain, vcpkg_triplet,
                     use_system_libs, no_manifest_mode, threads):
    """Generate compile_commands.json for C++ SDK."""
    manifest_mode = not no_manifest_mode
    ctx.obj.build_options.process_args(
        cmake_bin=cmake_bin, vcpkg_toolchain=vcpkg_toolchain,
        vcpkg_triplet=vcpkg_triplet, manifest_mode=manifest_mode,
        use_system_libs=use_system_libs, threads=threads)
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)
    ctx.obj.build_libs.check_for_python_lib("pybind11")
    ctx.obj.build_options.run_vcpkg_initialized_check()

    if output is None:
        output = os.path.join(ctx.obj.sdk_dir, "compile_commands.json")
    compile_commands_dir = os.path.join(ctx.obj.cmake_build_dir,
                                        "compile_commands")
    try:
        toolchain = None
        triplet = None
        if not ctx.obj.build_options.use_system_libs:
            toolchain = ctx.obj.build_options.vcpkg_toolchain
            triplet = ctx.obj.build_options.vcpkg_triplet
        ctx.obj.build_libs.generate_compile_commands(output,
                                                     ctx.obj.sdk_dir,
                                                     ctx.obj.sdk_artifact_dir,
                                                     compile_commands_dir,
                                                     toolchain=toolchain,
                                                     triplet=triplet,
                                                     env=ctx.obj.build_options.build_env,
                                                     cmake_path=ctx.obj.build_options.cmake_bin,
                                                     manifest_mode=ctx.obj.build_options.manifest_mode)
    except Exception:
        print("Error generating compile_commands.json")
        raise RuntimeError("Please check the output for details.")


def import_module(click_context):
    click_context.build_group.add_command(cpp_build)
    click_context.test_group.add_command(cpp_test)
    click_context.build_group.add_command(compile_commands)


def finalize(click_context):
    pass
