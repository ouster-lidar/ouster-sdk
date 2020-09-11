# Install Chocolatey
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))

# Install required utilities
choco install -y git visualstudio2019community visualstudio2019-workload-nativedesktop 7zip python3

# Refresh the current powershell environment so we can use the installed utilities2$env:ChocolateyInstall = Convert-Path "$((Get-Command choco).Path)\..\.."   
Set-ExecutionPolicy Bypass -Scope Process -Force; Import-Module "$env:ChocolateyInstall\helpers\chocolateyProfile.psm1"
refreshenv

# Install python requirements
pip install pytest wheel

# Install and build the library requirements
cd C:\
git clone https://github.com/Microsoft/vcpkg.git
& 'C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat'
cd vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg integrate install
.\vcpkg install --triplet x64-windows-dynamic glfw3 glew tclap jsoncpp eigen3 pybind11 libtins
