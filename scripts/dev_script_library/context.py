import os
import glob
import importlib
import sys
from pathlib import Path
import platform


class ClickContext:
    class BuildOptions:
        def __init__(self, vcpkg_dir, cmake_bin="cmake", vcpkg_toolchain=None,
                     vcpkg_triplet=None, threads=None,
                     manifest_mode=True):
            self._vcpkg_dir_getter = vcpkg_dir if callable(vcpkg_dir) else lambda: vcpkg_dir
            self.vcpkg_toolchain = None
            self.vcpkg_triplet = None
            self.use_system_libs = False
            self.vcpkg_cache_handler = None
            self.coverage_flags = False
            self.build_env = os.environ.copy()
            self.process_args(cmake_bin, vcpkg_toolchain, vcpkg_triplet,
                              threads, manifest_mode)

        @property
        def vcpkg_dir(self):
            """Get vcpkg directory"""
            return self._vcpkg_dir_getter()

        def process_args(self, cmake_bin="cmake", vcpkg_toolchain=None,
                         vcpkg_triplet=None, threads=None, manifest_mode=True,
                         use_system_libs=False, coverage_flags=False):
            """ Centralized spot for processing args for vcpkg and cmake
            and setting defaults """
            if coverage_flags and platform.system().lower() != "linux":
                raise RuntimeError("Coverage flags are not supported on anything but linux.")
            self.coverage_flags = coverage_flags
            if self.coverage_flags:
                self.build_env["CMAKE_COVERAGE_TESTS"] = "true"
            if vcpkg_toolchain is None:
                if os.path.exists(self.vcpkg_dir) and os.path.isdir(self.vcpkg_dir):
                    if os.path.exists(os.path.join(self.vcpkg_dir,
                                                   "scripts", "buildsystems",
                                                   "vcpkg.cmake")):
                        vcpkg_toolchain = os.path.join(self.vcpkg_dir,
                                                       "scripts", "buildsystems",
                                                       "vcpkg.cmake")
                    elif "VCPKG_ROOT" in self.build_env:
                        vcpkg_toolchain = os.path.join(self.build_env["VCPKG_ROOT"],
                                                       "scripts", "buildsystems",
                                                       "vcpkg.cmake")
            else:
                vcpkg_root_path = os.path.dirname(os.path.abspath(vcpkg_toolchain))
                vcpkg_root_path = os.path.abspath(os.path.join(vcpkg_root_path,
                                                               "..", ".."))
                vcpkg_triplet_folder = os.path.join(vcpkg_root_path,
                                                    "triplets")
                if not os.path.exists(vcpkg_triplet_folder):
                    # we have a pure installed vcpkg folder
                    # dont use manifest mode
                    manifest_mode = False
            if vcpkg_triplet is None:
                vcpkg_triplet = self._process_triplet()
            self.cmake_bin = cmake_bin
            self.vcpkg_toolchain = vcpkg_toolchain
            self.vcpkg_triplet = vcpkg_triplet
            if threads is None:
                self.threads = int(max(1, os.cpu_count() / 2))
            else:
                self.threads = threads
            self.build_env["VCPKG_MAX_CONCURRENCY"] = str(self.threads)
            self.manifest_mode = manifest_mode
            self.use_system_libs = use_system_libs
            if self.vcpkg_cache_handler is not None:
                self.vcpkg_cache_handler()

        def _process_triplet(self):
            arch = ""
            if platform.machine() == "aarch64" or platform.machine() == "arm64":
                arch = "arm64"
            elif platform.machine() == "x86_64":
                arch = "x64"
            if platform.system() == "Windows":
                triplet = f"{arch}-windows-static-md"
            elif platform.system() == "Linux":
                triplet = f"{arch}-linux"
            elif platform.system() == "Darwin":
                triplet = f"{arch}-osx"
            return triplet

        def run_vcpkg_initialized_check(self, alt_condition=False, local_vcpkg_only=False):
            first_condition = self.vcpkg_toolchain is None and self.use_system_libs is False
            if (first_condition or alt_condition):
                def vcpkg_system_deps_needed(prefix):
                    print(f"{prefix}NOTE: vcpkg needs some system dependencies to be installed for building.")
                    print(f"{prefix}On ubuntu run `sudo apt-get install flex bison libxinerama-dev "
                          "libxcursor-dev xorg-dev libglu1-mesa-dev pkg-config`")
                    print(f"{prefix}or use ./scripts/dev.sh utils install-vcpkg-package-requirements")

                script_prefix = sys.argv[0]
                if sys.argv[0].split(".")[-1] == "py":
                    script_prefix = f"python3 {sys.argv[0]}"
                print("ERROR: No source of dependencies set.")
                if local_vcpkg_only:
                    print("Please do the following:")
                else:
                    print("Please either:")
                print(f"\t* Install a local vcpkg repo using `{script_prefix} utils "
                      "enable-local-vcpkg` [RECOMENDED WAY].")
                vcpkg_system_deps_needed("\t\t")
                if not local_vcpkg_only:
                    print("\t* Set the VCPKG_ROOT environment variable to the vcpkg root directory.")
                    print("\t* Set the --vcpkg-toolchain argument to the vcpkg toolchain file on the command.")
                    print("\t* Set the --use-system-libs argument on the command.")
                    print("\t\tNOTE: This will use system libraries to build the sdk.")
                    print("\t\tYou may need to use your system package manager to install the dependencies.")
                    print("\t\tPlease refer to the ouster sdk documentation for more information.")
                raise RuntimeError("No vcpkg toolchain or system libraries set. ")

    def __init__(self, build_libs):
        self.top_level_group = None
        self.dev_dir = os.path.dirname(os.path.abspath(__file__))
        self.sdk_dir = os.path.abspath(os.path.join(self.dev_dir, '..', '..'))
        self.sdkx_dir = os.path.abspath(os.path.join(self.sdk_dir,
                                                     'sdk-extensions'))
        self.sdkx_dev_dir = os.path.abspath(os.path.join(self.sdkx_dir,
                                                         'scripts',
                                                         "dev_script_library"))
        self.sdk_build_dir = os.path.abspath(os.path.join(self.sdk_dir,
                                                          'build', "dev"))
        self._sdk_artifact_dir = os.path.abspath(os.path.join(self.sdk_dir,
                                                             'artifacts', "dev"))
        self._cmake_build_dir = os.path.abspath(os.path.join(self.sdk_build_dir,
                                                            "cmake"))
        self._docs_build_dir = os.path.abspath(os.path.join(self.sdk_build_dir,
                                                           "docs"))
        self._python_build_dir = os.path.abspath(os.path.join(self.sdk_build_dir,
                                                             "python"))
        self.sdk_test_dir = os.path.abspath(os.path.join(self.sdk_dir,
                                                         'build', "dev", "test"))
        self.vcpkg_json_file = os.path.join(self.sdk_dir, 'vcpkg.json')
        if "DEV_PERSISTANT_DIR" in os.environ:
            print(f"Using DEV_PERSISTANT_DIR from environment: {os.environ['DEV_PERSISTANT_DIR']}"),
            self._dev_persistant_dir = os.environ["DEV_PERSISTANT_DIR"]
        else:
            self._dev_persistant_dir = os.path.join(Path.home(), '.osdkv2')
        self._vcpkg_dir = os.path.join(self._dev_persistant_dir, 'vcpkg')
        self.submodules = {}
        # include this in the context for plugins not in this folder to have access to this
        self.build_libs = build_libs
        self.sdk_source_dirs = glob.glob(os.path.join(self.sdk_dir, "ouster_*"))
        self._build_options = None

        self.internal_test_data_dir = None
        self.internal_test_data_tag = None

    @property
    def vcpkg_dir(self):
        """Create vcpkg directory on first access"""
        # This will trigger dev_persistant_dir creation first
        # in case it doesn't exist
        self.dev_persistant_dir
        return self._vcpkg_dir

    @property
    def build_options(self):
        """Initialization of BuildOptions"""
        if self._build_options is None:
            # Pass a lambda that returns vcpkg_dir to avoid immediate creation
            self._build_options = self.BuildOptions(lambda: self.vcpkg_dir)
        return self._build_options

    @property
    def cmake_build_dir(self):
        """Create CMake build directory"""
        if not os.path.isdir(self._cmake_build_dir):
            os.makedirs(self._cmake_build_dir)
        return self._cmake_build_dir

    @property
    def docs_build_dir(self):
        """Create docs build directory"""
        if not os.path.isdir(self._docs_build_dir):
            os.makedirs(self._docs_build_dir)
        return self._docs_build_dir

    @property
    def python_build_dir(self):
        """Create Python build directory"""
        if not os.path.isdir(self._python_build_dir):
            os.makedirs(self._python_build_dir)
        return self._python_build_dir

    @property
    def dev_persistant_dir(self):
        """Create DEV_PERSISTANT_DIR"""
        if not os.path.isdir(self._dev_persistant_dir):
            os.makedirs(self._dev_persistant_dir)
        return self._dev_persistant_dir

    @property
    def sdk_artifact_dir(self):
        """Create SDK artifact directory"""
        if not os.path.isdir(self._sdk_artifact_dir):
            os.makedirs(self._sdk_artifact_dir)
        return self._sdk_artifact_dir

    def print_output_location(self, item):
        print(f"Output files for: \"{item}\" can be located in: {self.sdk_artifact_dir}")

    def add_module(self, module_file):
        module_dir = os.path.dirname(module_file)
        module_file = os.path.basename(module_file)
        module_name = module_file.replace(".py", "")
        if module_dir not in sys.path:
            sys.path.append(module_dir)
        module = importlib.import_module(module_name)
        self.submodules[module_name] = module
        module.import_module(self)

    def finalize(self):
        for item in self.submodules.values():
            item.finalize(self)
