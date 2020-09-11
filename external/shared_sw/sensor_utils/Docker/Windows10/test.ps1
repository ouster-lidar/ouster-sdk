$env:PATH = $env:PATH -replace ";;",";"
$env:PATH = $env:PATH -replace '"',''

Set-ExecutionPolicy Bypass -Scope Process -Force

# Make sure we error out on errors
$ErrorActionPreference = "Stop"

# For some reason 'pip install' does not deal with wildcards,
# so find the full path for 'pip install'
$current_dir = $($(Get-Location).Path)

$client_wheel_path = $($(Resolve-Path $current_dir\build\ouster_client*.whl).Path)
$pcap_wheel_path = $($(Resolve-Path $current_dir\build\ouster_pcap*.whl).Path)

# Set up the path for grabbing the test files from google cloud
$gs_source = "gs://artifacts.ouster-build.appspot.com"
$gs_source = "$gs_source/jenkins/test-files/ouster_pyclient"
$gs_target = "$current_dir/sensor_utils/test/test-files"

# Ignore "directory already exists errors"
mkdir $gs_target -ErrorAction SilentlyContinue

# For some reason gcloud emits an error to tell you that it successfully activated creds.
# # Due to this, temporarily disable erroring out.
# $ErrorActionPreference = "SilentlyContinue"
# gcloud auth activate-service-account --key-file=/Users/ContainerAdministrator/.docker/ouster-build-c3e7685e414d.json
# $ErrorActionPreference = "Stop"

# Grab the test files
gsutil -q rsync -d $gs_source $gs_target
if($LASTEXITCODE -ne 0) {
    throw "Build Failed"
}

rm -Recurse -Force -ErrorAction SilentlyContinue venv

python -m venv venv
. .\venv\Scripts\Activate.ps1
$venv_dir = $($(Get-Location).Path) + "/venv"
pip install wheel pytest
pip install $client_wheel_path
pip install $pcap_wheel_path
cd sensor_utils/test

# Uncomment these for a debug build and comment below
# cp C:/vcpkg/installed/x64-windows/debug/bin/Packet.dll $venv_dir/Lib/site-packages/ouster/
# cp C:/vcpkg/installed/x64-windows/debug/bin/wpcap.dll $venv_dir/Lib/site-packages/ouster/
# cp C:/vcpkg/installed/x64-windows/debug/bin/tins.dll $venv_dir/Lib/site-packages/ouster/
# cp C:/vcpkg/installed/x64-windows/debug/bin/jsoncpp.dll $venv_dir/Lib/site-packages/ouster/
# cp C:/vcpkg/installed/x64-windows/debug/bin/libcrypto*x64.dll $venv_dir/Lib/site-packages/ouster/

cp C:/vcpkg/installed/x64-windows/bin/Packet.dll $venv_dir/Lib/site-packages/ouster/
cp C:/vcpkg/installed/x64-windows/bin/wpcap.dll $venv_dir/Lib/site-packages/ouster/
cp C:/vcpkg/installed/x64-windows/bin/tins.dll $venv_dir/Lib/site-packages/ouster/
cp C:/vcpkg/installed/x64-windows/bin/jsoncpp.dll $venv_dir/Lib/site-packages/ouster/
cp C:/vcpkg/installed/x64-windows/bin/libcrypto*x64.dll $venv_dir/Lib/site-packages/ouster/


# Run the tests
python -m pytest --junitxml=test.xml -s
if($LASTEXITCODE -ne 0) {
    throw "Build Failed"
}
