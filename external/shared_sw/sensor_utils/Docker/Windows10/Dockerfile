# escape=`
FROM msvs-base

# Install any other smaller dependencies here
# Examples:
# Choco: https://chocolatey.org/ General "apt-get" for windows
# RUN choco install -y git python3
# Pip: python package manager
# RUN pip install pytest wheel
# VCPKG: Visual Studio Library Package Command
# NOTE: Always use the x64-windows-static triplet
# RUN \BuildTools\Common7\Tools\VsDevCmd.bat && `
#	\vcpkg\vcpkg install --triplet x64-windows-static eigen3

RUN \BuildTools\Common7\Tools\VsDevCmd.bat && \vcpkg\vcpkg update
RUN \BuildTools\Common7\Tools\VsDevCmd.bat && `
        \vcpkg\vcpkg install --triplet x64-windows-static eigen3 && `
        \vcpkg\vcpkg install --triplet x64-windows-static jsoncpp && `
        \vcpkg\vcpkg install --triplet x64-windows-static pybind11 && `
        \vcpkg\vcpkg install --triplet x64-windows-static tclap

# Set shell for chocolatey installs
SHELL ["powershell.exe", "-NoLogo", "-ExecutionPolicy", "Bypass"]

RUN     curl -OutFile /TEMP/Win10Pcap.msi http://www.win10pcap.org/download/Win10Pcap-v10.2-5002.msi; `
        if ((Get-FileHash /TEMP/Win10Pcap.msi).hash -ne 'CE1169C7CAC4BC9BC45E159CEC069F0AB57C42FC3F636456A2E404CC6B91E855') `
        { `
            write-error 'Winpcap DLL Verification Error'; `
            throw 'Winpcap DLL Verification Error'; `
        } `
        else `
        { `
            choco install -y 7zip; `
            write-host 'Installing winpcap dlls'; `
            cd \temp; `
            7z x Win10Pcap.msi; `
            7z x Win10Pcap.cab; `
            mv DIR_x64_Packet_dll \Windows\System32\Packet.dll; `
            mv DIR_x64_wpcap_dll \Windows\System32\wpcap.dll; `
            write-host 'Registering winpcap dlls'; `
            regsvr32 Packet.dll; `
            regsvr32 wpcap.dll; `
            write-host 'Finished installing winpcap dlls'; `
        }

RUN pip install pytest

# Restore the default Windows shell for correct batch processing.
SHELL ["cmd", "/S", "/C"]

RUN powershell "curl -OutFile C:\TEMP\WpdPack.zip https://www.winpcap.org/install/bin/WpdPack_4_1_2.zip; `
    if ((Get-FileHash C:\TEMP\WpdPack.zip).hash -ne 'EA799CF2F26E4AFB1892938070FD2B1CA37CE5CF75FEC4349247DF12B784EDBD') { `
        write-error 'Winpcap Verification Error';`
        throw 'Winpcap Verification Error'`
    } `
    else {`
        write-host 'Installing winpcap';`
        cd \temp;`
        Expand-Archive C:\TEMP\WpdPack.zip; `
        cp -Recurse -Force C:\Temp\WpdPack\WpdPack \`
    }"

ENTRYPOINT C:\BuildTools\Common7\Tools\VsDevCmd.bat &&

