import os
import re
from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
from conan.tools.files import collect_libs, load
from conan.tools.scm import Git


class ousterSdkRecipe(ConanFile):
    name = "ouster_sdk"
    package_type = "library"
    license = "BSD 3-Clause License"
    author = "Ouster, Inc."
    url = "https://github.com/ouster-lidar/ouster-sdk"
    description = "Ouster SDK - tools for working with Ouster Lidars"
    topics = ("lidar", "driver", "hardware", "point cloud", "3d", "robotics", "automotive")
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "build_viz": [True, False],
        "build_pcap": [True, False],
        "build_osf": [True, False],
        "shared": [True, False],
        "fPIC": [True, False],
        "ensure_cpp17": [True, False],
        "eigen_max_align_bytes": [True, False],
    }
    default_options = {
        "build_viz": False,
        "build_pcap": False,
        "build_osf": False,
        "shared": False,
        "fPIC": True,
        "ensure_cpp17": False,
        "eigen_max_align_bytes": False,
    }

    exports_sources = [
        "cmake/*",
        "conan/*",
        "ouster_client/*",
        "ouster_pcap/*",
        "ouster_osf/*",
        "ouster_viz/*",
        "tests/*",
        "thirdparty/*",
        "CMakeLists.txt",
        "CMakeSettings.json",
        "LICENSE",
        "LICENSE-bin",
        "README.rst"
    ]

    # https://docs.conan.io/2/reference/conanfile/methods/set_version.html
    def set_version(self):
        content = load(self, os.path.join(self.recipe_folder, "CMakeLists.txt"))
        version = re.search("set\(OusterSDK_VERSION_STRING (.*)\)", content).group(1)
        self.version = version.strip()

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def requirements(self):
        # not required directly here but because boost and openssl pulling
        # slightly different versions of zlib we need to set it
        # here explicitly
        self.requires("zlib/1.3")

        # Since Eigen is a header only library, and the SDK includes Eigen
        # headers in its headers, we must set transitive_headers=True so that
        # packages consuming the SDK will also have access to the Eigen headers.
        self.requires("eigen/3.4.0", transitive_headers=True)
        self.requires("jsoncpp/1.9.5")
        self.requires("spdlog/1.12.0")
        self.requires("fmt/9.1.0", override=True)
        self.requires("libcurl/7.86.0")

        if self.options.build_pcap:
            self.requires("libtins/4.3")

        if self.options.build_osf:
            self.requires("flatbuffers/23.5.26")
            self.requires("libpng/1.6.39")

        if self.options.build_viz:
            self.requires("glad/0.1.34")
            if self.settings.os != "Windows":
                self.requires("xorg/system")
            self.requires("glfw/3.3.6")
            # maybe needed for cpp examples, but not for the lib
            # self.requires("tclap/1.2.4")

    def build_requirements(self):
        if self.options.build_osf:
            self.build_requires("flatbuffers/<host_version>")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.variables["BUILD_VIZ"] = self.options.build_viz
        tc.variables["BUILD_PCAP"] = self.options.build_pcap
        tc.variables["BUILD_OSF"] = self.options.build_osf
        tc.variables[
            "OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32"
        ] = self.options.eigen_max_align_bytes
        tc.variables["BUILD_SHARED_LIBS"] = True if self.options.shared else False
        tc.variables["CMAKE_POSITION_INDEPENDENT_CODE"] = (
            True if "fPIC" in self.options and self.options.fPIC else False
        )

        # we use this option until we remove nonstd::optional from SDK codebase (soon)
        if self.options.ensure_cpp17:
            tc.variables["CMAKE_CXX_STANDARD"] = 17

        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = collect_libs(self)
        self.cpp_info.includedirs = [
            "include",
            "include/optional-lite"
        ]
        self.cpp_info.build_modules["cmake_find_package"].append(
            "lib/cmake/OusterSDK/OusterSDKConfig.cmake"
        )

        self.cpp_info.set_property(
            "cmake_build_modules",
            [os.path.join("lib", "cmake", "OusterSDK", "OusterSDKConfig.cmake")],
        )
        self.cpp_info.set_property("cmake_file_name", "OusterSDK")
        self.cpp_info.set_property("cmake_target_name", "OusterSDK")

        # TODO: to remove in conan v2 once cmake_find_package* generators removed
        self.cpp_info.filenames["cmake_find_package"] = "OusterSDK"
        self.cpp_info.filenames["cmake_find_package_multi"] = "OusterSDK"
        self.cpp_info.names["cmake_find_package"] = "OusterSDK"
        self.cpp_info.names["cmake_find_package_multi"] = "OusterSDK"
