$env:PATH = "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools;C:\PROGRAM FILES (X86)\MICROSOFT VISUAL STUDIO\2019\COMMUNITY\COMMON7\IDE\COMMONEXTENSIONS\MICROSOFT\CMAKE\CMake\bin\;C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\MSBuild\Current\Bin\amd64;$env:PATH"

# Build the client wheel
$env:CMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
$env:CMAKE_PREFIX_PATH=$(Get-Location).Path + "\ouster_example"
pip wheel --no-deps .\python-client -w .\build
if($LASTEXITCODE -ne 0) {
    throw "Build Failed"
}
$env:CMAKE_PREFIX_PATH=$env:CMAKE_PREFIX_PATH + ";" + $(Get-Location).Path + "\ouster_pcap"
pip wheel --no-deps .\ouster_pcap\python-pcap -w .\build
if($LASTEXITCODE -ne 0) {
    throw "Build Failed"
}
cd build

# Build the visual studio solution
msbuild ALL_BUILD.vcxproj /p:Configuration=Release
if($LASTEXITCODE -ne 0) {
    throw "Build Failed"
}

cd ..
