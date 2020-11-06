# Requirements
- Visual Studio Installed [Download Here](https://visualstudio.microsoft.com/downloads/)
- Visual Studio CMake Support [Doc Here](https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019)
- Visual Studio CPP Support [Doc Here](https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=vs-2019)

# Dependency Install
1. Download vcpkg [Download Here](https://github.com/microsoft/vcpkg/archive/master.zip)
    - vcpkg is a dependency manager developed by microsoft, [more info located here](https://github.com/microsoft/vcpkg)
2. Extract the vcpkg zip file to somewhere on your filesystem
3. Open 'Developer Command Prompt for VS' from start menu
4. Run the following steps in the command window (Make sure visual studio is not open during this) [Doc Here](https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019)
    1. `cd <location of the extracted vcpkg folder>`
        - Replace <location of the extracted vcpkg folder> with the path of the extracted vcpkg folder
    2. `bootstrap-vcpkg.bat`
        - This will start building the vcpkg framework
    3. `.\vcpkg integrate install`
        - This will integrate vcpkg with your Visual Studio IDE
    4. `.\vcpkg install --triplet x64-windows glfw3 glew tclap jsoncpp eigen3`
        - This will build and install the dependencies
        - **NOTE: This will take a while**

# Opening The Project
1. Start Visual Studio
2. When the prompt opens asking you what type of project to open click 'Open a local folder'
3. Open one of the following
    1. ouster_client
        - This is just the generic example program
    2. ouster_viz
        - This is the example visualizer
4. Make sure Visual Studio is building in release mode [Doc Here](https://docs.microsoft.com/en-us/visualstudio/debugger/how-to-set-debug-and-release-configurations?view=vs-2019)
5. Go to Build -> Build All
    - **NOTE: If 'Build All' is not under the build menu there may have been a cmake issue**
    - **look at the error output to see if there is an error**
    - **you may need to regenerate the cmake cache by going to the following**
    - Project -> Generate cache for ....
6. To run make sure you allow the client/visualizer through the windows firewall
