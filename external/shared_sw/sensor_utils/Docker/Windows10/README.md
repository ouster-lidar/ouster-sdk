# How To Install And Compile
## Initial Steps
Copy the shared_sw code to the windows computer
## Install
Open up an admin powershell console
Run the following:
``
PowerShell.exe -ExecutionPolicy UnRestricted -File <location of shared_sw>\sensor_utils\Docker\Windows10\install_windows_toolchain.ps1
``
Close the admin console after the install is finished
## Compiling
Open up a regular powershell console
Change directory to shared sw
``
cd <location of shared_sw>
``
Run the following to generate the project:
``
.\sensor_utils\Docker\Windows10\make_vs_project.bat
``
Run the following to compile the project:
``
.\sensor_utils\Docker\Windows10\build.bat
``
