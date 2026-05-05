import click
import sys
import os
import shutil

additional_functionality_callback = None


def detect_package_manager():
    if shutil.which("apt-get"):
        return "apt"
    elif shutil.which("yum"):
        return "yum"
    elif shutil.which("dnf"):
        return "yum"
    elif shutil.which("pacman"):
        return "pacman"
    elif shutil.which("zypper"):
        return "zypper"
    elif shutil.which("brew"):
        return "brew"
    elif shutil.which("apk"):
        return "alpine"
    else:
        return None


@click.command()
@click.pass_context
@click.option('--reinit',
              default=False,
              is_flag=True,
              help='Reinitialize vcpkg.')
def enable_local_vcpkg(ctx, reinit):
    """Enable local vcpkg repo."""
    ctx.obj.build_libs.initialize_vcpkg(ctx.obj.vcpkg_dir, reinit)
    if additional_functionality_callback is not None:
        additional_functionality_callback(ctx)


def install_vcpkg_package_requirements_apt(ctx):
    packages = [
        "git", "build-essential", "cmake",
        "curl", "zip", "unzip", "tar",
        "pkg-config", "libxcursor-dev",
        "libxinerama-dev", "libgl1-mesa-dev",
        "xorg-dev", "bison", "flex", "flufl.lock"
    ]
    run = ctx.obj.build_libs.RunCommand(tty=True)
    run.run_command("apt-get", "update")
    args = ["apt-get", "install", "-y"]
    args.extend(packages)
    run.run_command(*args)


def install_vcpkg_package_requirements_brew(ctx):
    packages = [
        'curl', 'git', 'cmake', 'pkg-config', "flufl.lock"
    ]
    run = ctx.obj.build_libs.RunCommand(tty=True)
    run.run_command("brew", "update")
    args = ["brew", "install"]
    args.extend(packages)
    run.run_command(*args)
    pass


@click.command()
@click.pass_context
def install_vcpkg_package_requirements(ctx):
    """Install local vcpkg dependencies."""
    if os.name == "nt":
        print("No vcpkg depenendencies required.\n")
        sys.exit(0)
    else:
        pm = detect_package_manager()
        if pm == "apt":
            install_vcpkg_package_requirements_apt(ctx)
        elif pm == "brew":
            install_vcpkg_package_requirements_brew(ctx)
        else:
            print("No vcpkg depenendencies required.\n")
            sys.exit(0)


def install_doxygen_linux(ctx, version="1.11.0"):
    url = f"https://www.doxygen.nl/files/doxygen-{version}.linux.bin.tar.gz"
    tarball = f"doxygen-{version}.linux.bin.tar.gz"
    run = ctx.obj.build_libs.RunCommand(tty=True)
    run.run_command("wget", url)
    run.run_command("tar", "-zxf", tarball)
    # Find doxygen* directory
    dirs = [d for d in os.listdir(".") if d.startswith("doxygen") and os.path.isdir(d)]
    if not dirs:
        raise RuntimeError("Doxygen directory not found after extraction.")
    doxygen_dir = dirs[0]
    run.run_command("make", cwd=doxygen_dir)
    run.run_command("make", "install", cwd=doxygen_dir)
    run.run_command("ln", "-sf", "/usr/local/bin/doxygen", "/usr/bin/doxygen")


def install_system_packages_apt(ctx, doxygen=False, clangformat=False):
    packages = [
        'libeigen3-dev', 'libtins-dev',
        'libpcap-dev', 'libcurl4-openssl-dev',
        'git', 'build-essential', 'cmake',
        'zlib1g-dev', 'libglfw3-dev',
        'libpng-dev', 'libflatbuffers-dev',
        'libtbb-dev', 'robin-map-dev',
        'libceres-dev', 'libzstd-dev',
        'libzip-dev', 'libssl-dev',
        'libgtest-dev'
    ]
    if doxygen:
        packages.append('graphviz')
        install_doxygen_linux(ctx)
    if clangformat:
        packages.append('clang-format')
    run = ctx.obj.build_libs.RunCommand(tty=True)
    run.run_command("apt-get", "update")
    args = ["apt-get", "install", "-y"]
    args.extend(packages)
    run.run_command(*args)


def install_system_packages_brew(ctx, doxygen=False, clangformat=False):
    packages = [
        'eigen', 'libtins', 'libpcap',
        'curl', 'git', 'cmake', 'zlib',
        'glfw', 'libpng', 'flatbuffers',
        'tbb', 'ceres-solver', 'zstd',
        'robin-map', 'libzip', 'openssl@3',
        'googletest'
    ]
    if doxygen:
        packages.append('doxygen')
    if clangformat:
        packages.append('clang-format')
    run = ctx.obj.build_libs.RunCommand(tty=True)
    run.run_command("brew", "update")
    args = ["brew", "install"]
    args.extend(packages)
    run.run_command(*args)


@click.command()
@click.pass_context
@click.option('--doxygen', default=False,
              is_flag=True, help='Enable doxygen install with other system packages.')
@click.option('--clangformat', default=False,
              is_flag=True, help='Enable clang-format install with other system packages.')
def install_system_packages(ctx, doxygen, clangformat):
    """Install system packages for your platform."""
    if os.name == "nt":
        print("Windows is detected, no binary system package manager is supported. "
              "Please use dev.py utils enable-local-vcpkg for this distribution/OS.\n")
        sys.exit(1)
    else:
        pm = detect_package_manager()
        if pm == "apt":
            install_system_packages_apt(ctx, doxygen, clangformat)
        elif pm == "brew":
            install_system_packages_brew(ctx, doxygen, clangformat)
        else:
            print(f"ERROR: This distribution/OS is not officially supported. ({pm})\n"
                  "Please use dev.py utils enable-local-vcpkg for this distribution/OS.\n")
            sys.exit(1)


def import_module(click_context):
    click_context.utils_group.add_command(enable_local_vcpkg)
    click_context.utils_group.add_command(install_system_packages)
    click_context.utils_group.add_command(install_vcpkg_package_requirements)


def finalize(click_context):
    pass
