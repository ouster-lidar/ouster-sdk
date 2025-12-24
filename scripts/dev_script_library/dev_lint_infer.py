import click
import os
import shutil


@click.command()
@click.pass_context
@click.option('--infer-bin',
              default="infer",
              help='Pass in alternative infer binary.')
@click.option('--output-dir',
              default=None,
              help='Output directory for the infer analysis.')
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
              default=os.cpu_count(),
              help='Number of threads to use.')
@click.option('--no-manifest-mode',
              default=False,
              is_flag=True,
              help='Disable vcpkg manifest mode.')
def infer_cpp_static_analysis(ctx, infer_bin, output_dir, cmake_bin,
                              vcpkg_toolchain, vcpkg_triplet, threads,
                              use_system_libs, no_manifest_mode):
    """ Static analysis for C++ code using Infer."""
    manifest_mode = not no_manifest_mode
    ctx.obj.build_options.process_args(
        cmake_bin=cmake_bin, vcpkg_toolchain=vcpkg_toolchain,
        vcpkg_triplet=vcpkg_triplet, threads=threads,
        manifest_mode=manifest_mode, use_system_libs=use_system_libs)
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)

    ctx.obj.build_options.run_vcpkg_initialized_check()

    ctx.obj.build_libs.check_for_python_lib("pybind11")

    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)
    ctx.obj.build_libs.check_for_tool(infer_bin)

    manifest_mode = False
    infer_commands_dir = os.path.join(ctx.obj.cmake_build_dir,
                                      "infer")
    compile_commands_json = os.path.join(infer_commands_dir,
                                         "compile_commands.json")
    print(f"Manifest Mode: {manifest_mode}")
    try:
        toolchain = None
        triplet = None
        if not ctx.obj.build_options.use_system_libs:
            toolchain = ctx.obj.build_options.vcpkg_toolchain
            triplet = ctx.obj.build_options.vcpkg_triplet
        ctx.obj.build_libs.generate_compile_commands(compile_commands_json,
                                                     ctx.obj.sdk_dir,
                                                     ctx.obj.sdk_artifact_dir,
                                                     infer_commands_dir,
                                                     toolchain=toolchain,
                                                     triplet=triplet,
                                                     cmake_path=ctx.obj.build_options.cmake_bin,
                                                     manifest_mode=ctx.obj.build_options.manifest_mode,
                                                     env=ctx.obj.build_options.build_env)
    except Exception:
        print("Error generating compile_commands.json")
        raise RuntimeError("Please check the output for details.")

    if output_dir is None:
        output_dir = ctx.obj.sdk_artifact_dir

    run = ctx.obj.build_libs.RunCommand(tty=True)
    args = [infer_bin, "--compilation-database",
            compile_commands_json, "--cost",
            "--bufferoverrun", "--racerd",
            "--dump-duplicate-symbols",
            "--loop-hoisting", "--pulse",
            "--topl", "--siof", "--starvation",
            "-j", str(ctx.obj.build_options.threads)]
    run.run_command(*args, cwd=infer_commands_dir)
    if output_dir is not None:
        shutil.move(os.path.join(infer_commands_dir, "infer-out"),
                    os.path.join(output_dir, "infer-out"))


def import_module(click_context):
    click_context.lint_group.add_command(infer_cpp_static_analysis)


def finalize(click_context):
    pass
