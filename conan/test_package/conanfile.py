import os

from conans import ConanFile, CMake, tools


class OusterSDKTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake_paths", "cmake_find_package"

    def build(self):
        cmake = CMake(self)
        # Current dir is "test_package/build/<build_id>" and CMakeLists.txt is
        # in "test_package"
        cmake.definitions["BUILD_OSF"] = True
        cmake.definitions[
            "CMAKE_PROJECT_PackageTest_INCLUDE"] = os.path.join(
                self.build_folder, "conan_paths.cmake")
        cmake.configure()
        cmake.build()

    def imports(self):
        self.copy("*.dll", dst="bin", src="bin")
        self.copy("*.dylib*", dst="bin", src="lib")
        self.copy('*.so*', dst='bin', src='lib')

    def test(self):
        if not tools.cross_building(self):
            os.chdir("examples")
            # on Windows VS puts execulables under `./Release` or `./Debug` folders
            exec_path = f"{os.sep}{self.settings.build_type}" if self.settings.os == "Windows" else ""
            self.run(f".{exec_path}{os.sep}client_example")

            # List files + ldd for verbosity
            if self.settings.os == "Linux":
                self.run(f"ls -al .{exec_path}{os.sep}")
                self.run(f"ldd .{exec_path}{os.sep}osf_reader_example")

            # Smoke test OSF lib
            self.run(f".{exec_path}{os.sep}osf_reader_example")
