import os
import re
from conans import ConanFile, CMake, tools

from pprint import pformat

class OusterSDKConan(ConanFile):
    name = "ouster_sdk"
    license = "BSD 3-Clause License"
    author = "Ouster, Inc."
    url = "https://github.com/ouster-lidar/ouster_example"
    description = "Ouster SDK - tools for working with Ouster Lidars"
    topics = ("lidar", "driver", "hardware", "point cloud", "3d", "robotics", "automotive")
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "build_viz": [True, False],
        "build_pcap": [True, False],
        "shared": [True, False],
        "fPIC": [True, False],
        "ensure_cpp17": [True, False],
        "eigen_max_align_bytes": [True, False],
    }
    default_options = {
        "build_viz": False,
        "build_pcap": False,
        "shared": False,
        "fPIC": True,
        "ensure_cpp17": False,
        "eigen_max_align_bytes": False,
    }

    generators = "cmake_paths", "cmake_find_package"
    exports_sources = [
        "cmake/*",
        "conan/*",
        "ouster_client/*",
        "ouster_pcap/*",
        "ouster_viz/*",
        "tests/*",
        "CMakeLists.txt",
        "CMakeSettings.json",
        "LICENSE",
        "LICENSE-bin",
        "README.rst"
    ]

    # https://docs.conan.io/en/1.51/howtos/capture_version.html#how-to-capture-package-version-from-text-or-build-files
    def set_version(self):
        content = tools.load(os.path.join(self.recipe_folder, "CMakeLists.txt"))
        version = re.search("set\(OusterSDK_VERSION_STRING (.*)\)", content).group(1)
        self.version = version.strip()

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def requirements(self):
        # not required directly here but because libtins and libcurl pulling
        # slightly different versions of zlib and openssl we need to set it
        # here explicitly
        self.requires("zlib/1.2.13")
        self.requires("openssl/1.1.1s")

        self.requires("eigen/3.4.0")
        self.requires("jsoncpp/1.9.5")
        self.requires("spdlog/1.10.0")
        self.requires("libcurl/7.82.0")

        if self.options.build_pcap:
            self.requires("libtins/4.3")

        if self.options.build_viz:
            self.requires("glad/0.1.34")
            if self.settings.os != "Windows":
                self.requires("xorg/system")
            self.requires("glfw/3.3.6")
            # maybe needed for cpp examples, but not for the lib
            # self.requires("tclap/1.2.4")

    def configure_cmake(self):
        cmake = CMake(self)
        cmake.definitions["BUILD_VIZ"] = self.options.build_viz
        cmake.definitions["BUILD_PCAP"] = self.options.build_pcap
        cmake.definitions["OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32"] = self.options.eigen_max_align_bytes
        # alt way, but we use CMAKE_TOOLCHAIN_FILE in other pipeline so avoid overwrite
        # cmake.definitions["CMAKE_TOOLCHAIN_FILE"] = os.path.join(self.build_folder, "conan_paths.cmake")
        cmake.definitions[
            "CMAKE_PROJECT_ouster_example_INCLUDE"] = os.path.join(
                self.build_folder, "conan_paths.cmake")
        cmake.definitions["BUILD_SHARED_LIBS"] = True if self.options.shared else False
        cmake.definitions["CMAKE_POSITION_INDEPENDENT_CODE"] = (
            True if "fPIC" in self.options and self.options.fPIC else False
        )

        # we use this option until we remove nonstd::optional from SDK codebase (soon)
        if self.options.ensure_cpp17:
            cmake.definitions["CMAKE_CXX_STANDARD"] = 17

        self.output.info("Cmake definitions: ")
        self.output.info(pformat(cmake.definitions))
        cmake.configure()
        return cmake

    def build(self):
        cmake = self.configure_cmake()
        cmake.build()

    def package(self):
        cmake = self.configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
        self.cpp_info.includedirs = [
            "include",
            "include/optional-lite"
        ]
        self.cpp_info.build_modules["cmake_find_package"].append(
            "lib/cmake/OusterSDK/OusterSDKConfig.cmake"
        )

        self.cpp_info.set_property("cmake_file_name", "OusterSDK")

        self.cpp_info.filenames["cmake_find_package"] = "OusterSDK"
        self.cpp_info.filenames["cmake_find_package_multi"] = "OusterSDK"
        self.cpp_info.names["cmake_find_package"] = "OusterSDK"
        self.cpp_info.names["cmake_find_package_multi"] = "OusterSDK"
