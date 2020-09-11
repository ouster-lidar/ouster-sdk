$env:PATH = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools;C:\PROGRAM FILES (X86)\MICROSOFT VISUAL STUDIO\2019\COMMUNITY\COMMON7\IDE\COMMONEXTENSIONS\MICROSOFT\CMAKE\CMake\bin\;C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\amd64;$env:PATH"

# Remove any pre-existing build directories
rm -Recurse -Force -ErrorAction SilentlyContinue build

mkdir build
cd build
# Generate the visual studio solution from cmake
# The toolchain is for the build dependencies
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_GENERATOR_PLATFORM=x64 -DVCPKG_TARGET_TRIPLET="x64-windows" -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake" -DCMAKE_CXX_FLAGS="/W0" -DCMAKE_C_FLAGS="/W0" -DCMAKE_CXX_FLAGS_RELEASE="/W0" -DCMAKE_C_FLAGS_RELEASE="/W0" ..
if($LASTEXITCODE -ne 0) {
    throw "CMAKE Generation Failed"
}

cd ..
