import os

from conans import ConanFile, CMake, tools

class OusterSDKTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake_paths", "cmake_find_package"

    def build(self):
        cmake = CMake(self)
        # Current dir is "test_package/build/<build_id>" and CMakeLists.txt is
        # in "test_package"
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
            self.run(".%sclient_example" % os.sep)
