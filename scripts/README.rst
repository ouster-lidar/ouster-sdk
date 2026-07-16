======================
Ouster SDK Dev Scripts
======================

The Ouster SDK Dev Script Cli is a command-line interface designed to streamline common development tasks
such as building, testing, linting, and generating documentation for the Ouster SDK. Below is a detailed
guide to the available commands and their options.

---------------
Getting Started
---------------

^^^^^^^^^^^^^^^^^^^
Python Dependencies
^^^^^^^^^^^^^^^^^^^
The Ouster SDK Dev Script CLI requires the Python packages listed in
``scripts/requirements.txt``.  Install them with:

.. code-block:: bash

    python3 -m pip install -r scripts/requirements.txt


^^^^^^^^^^^^^^^^^^^^^^
CPP VCPKG Dependencies
^^^^^^^^^^^^^^^^^^^^^^

Below are the installation instructions for macOS and Linux.

### macOS
1. **Install Homebrew** (if not already installed):
   Homebrew is a package manager for macOS. You can install it using the following command:

   .. code-block:: bash

      /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

2. **Install Dependencies**:
   Use Homebrew to install the required dependencies:

   .. code-block:: bash

      brew install m4 bison pkg-config flex

### Linux
1. **Update Package Manager**:
   Update your system's package manager to ensure you have access to the latest versions of the dependencies:

   .. code-block:: bash

      sudo apt update

2. **Install Dependencies**:
   Use the package manager to install the required dependencies. For Ubuntu-based distributions:

   .. code-block:: bash

      sudo apt install -y flex bison libxinerama-dev libxcursor-dev xorg-dev libglu1-mesa-dev pkg-config build-essential


^^^^^^^^^^^
VCPKG Setup
^^^^^^^^^^^

Once the system-level build tools above are installed, use the Dev Script CLI to
install the remaining vcpkg package requirements and bootstrap a local vcpkg
checkout. On Linux/macOS run:

.. code-block:: bash

   # Install vcpkg package requirements via the system package manager
   # (apt on Debian/Ubuntu, brew on macOS). No-op on Windows.
   python3 ./scripts/dev.py utils install-vcpkg-package-requirements

   # Initialize the bundled vcpkg checkout so the SDK builds against it.
   python3 ./scripts/dev.py utils enable-local-vcpkg

Pass ``--reinit`` to ``enable-local-vcpkg`` to force re-initialization of the
local vcpkg repo:

.. code-block:: bash

   python3 ./scripts/dev.py utils enable-local-vcpkg --reinit


^^^^^^^^^^^^^^^^^^
Other Dependencies
^^^^^^^^^^^^^^^^^^
In addition to the python and cpp vcpkg dependencies, the Ouster SDK Dev Script Cli uses the following dependencies:

1. **CMake**: CMake is a cross-platform build system generator used to configure and build C++ projects.

   **Minimum Version**: CMake >= 3.15

2. **Doxygen**: Doxygen is a documentation generator for C++ and other languages.

   **Minimum Version**: Doxygen >= 1.10.0

3. **libclang**: libclang provides bindings for parsing and analyzing C++ code.

   **Minimum Version**: libclang >= 16.0.0

4. **clang-tidy**: clang-tidy is a C++ linter and static analysis tool.

   **Minimum Version**: clang-tidy >= 16.0.0

5. **clang-format**: clang-format is a tool for formatting C++ code according to style guidelines.

   **Minimum Version**: clang-format >= 16.0.0

6. **Facebook Infer**: Infer is a static analysis tool for detecting bugs in C++ and other languages.

Below are the installation instructions for macOS, Linux and Windows.

### macOS

1. **Install Homebrew** (if not already installed):
   Homebrew is a package manager for macOS. You can install it using the following command:

   .. code-block:: bash

      /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

2. **Install Dependencies**:
   Use Homebrew to install the required dependencies:

   .. code-block:: bash

      brew install cmake doxygen llvm infer

   - `llvm` includes `libclang`, `clang-tidy`, and `clang-format`.
   - Ensure the installed version of `llvm` is >= 16.0.0.

3. **Verify Installation**:
   Check the installed versions to ensure they meet the minimum requirements:

   .. code-block:: bash

      cmake --version
      doxygen --version
      clang --version
      clang-tidy --version
      clang-format --version
      infer --version

### Linux

1. **Update Package Manager**:
   Update your system's package manager to ensure you have access to the latest versions of the dependencies:

   .. code-block:: bash

      sudo apt update

2. **Install Dependencies**:
   Use the package manager to install the required dependencies. For Ubuntu-based distributions:

   .. code-block:: bash

      sudo apt install -y cmake doxygen clang clang-tidy clang-format

   For `libclang` and `clang` tools, ensure the installed version is >= 16.0.0. If your distribution does not provide the required version, you may need to install LLVM manually:

   .. code-block:: bash

      wget https://apt.llvm.org/llvm.sh
      chmod +x llvm.sh
      sudo ./llvm.sh 16

   For `doxygen` tools, ensure the installed version is >= 1.10.0. If your distribution does not provide the required version, you may need to install doxygen manually:

   a. Install build dependencies

   .. code-block:: bash
       sudo apt-get install build-essential cmake

   b. Download the latest version of Doxygen Source from the official website:

     `https://www.doxygen.nl/download.html`

   c. Extract the downloaded archive:

   .. code-block:: bash

      tar -xvf doxygen-<version>.src.tar.gz
      cd doxygen-<version>

   d. Build and install Doxygen:

   .. code-block:: bash

      mkdir build
      cd build
      cmake ..
      make
      sudo make install

   e. Verify the installation:

   .. code-block:: bash

      doxygen --version

   To install Facebook Infer, follow the official instructions from `https://github.com/facebook/infer`.

3. **Verify Installation**:
   Check the installed versions to ensure they meet the minimum requirements:

   .. code-block:: bash

      cmake --version
      doxygen --version
      clang --version
      clang-tidy --version
      clang-format --version
      infer --version

Windows

.. warning::
   Windows functionality is untested, please use at your own risk.

1. Install CMake:
   Download and install CMake from the official website:
   `https://cmake.org/download/`

   During installation, make sure you add CMake to your system's PATH.

2. Install LLVM (libclang, clang-tidy, clang-format):
   Download and install LLVM from the official website:
   `https://releases.llvm.org/`

   Ensure the installed version is >= 16.0.0. Add the LLVM binaries to your system's PATH.

3. Install Doxygen:
   Download and install Doxygen from the official website:
   `https://www.doxygen.nl/download.html`

   Verify the installed version is >= 1.10.0.

4. Facebook Infer:
   Facebook Infer is not natively supported on Windows. Windows infer static analysis will not be available.

5. Verify Installation:
   Open a terminal (Command Prompt or PowerShell) and check the installed versions:

   .. code-block:: powershell

      cmake --version
      doxygen --version
      clang --version
      clang-tidy --version
      clang-format --version

^^^^^^^^^^^^
Example Uses
^^^^^^^^^^^^

Build and testing the C++ SDK:

.. code-block:: bash

   python3 scripts/dev.py build cpp --threads 4 --build-type Debug
   python3 scripts/dev.py test cpp --threads 4 --build-type Debug

Building and testing Python:

.. code-block:: bash

   python3 scripts/dev.py build python
   python3 scripts/dev.py test python

Lint Python Code:

.. code-block:: bash

   python3 scripts/dev.py lint flake8

Generate Documentation:

.. code-block:: bash

   python3 scripts/dev.py build doxygen

^^^^^^^^^^^^^^^^^^^^^
Environment Variables
^^^^^^^^^^^^^^^^^^^^^

The Ouster SDK Dev Script Cli supports using environment variables for command options/arguments via Click.
This feature allows you to set default values for options without needing to specify them explicitly in every command invocation.
It is particularly useful for automating workflows or setting up consistent environments. The name of the environent variable
is prefixed with OSDK_DEV_CLI and is the option name uppercase with `-` replaced with `_`.

Below is an example using the `--threads` option.

1. **Setting the Environment Variable**:

   To set the number of threads using an environment variable, use the `export` command in your terminal:

   .. code-block:: bash

      export OSDK_DEV_CLI_THREADS=4

2. **Running the Cli**:

   When you run the CLI, the value of `OSDK_DEV_CLI_THREADS` will be used automatically for the `--threads` option:

   .. code-block:: bash

      python3 scripts/dev.py build cpp

   The above command will use 4 threads as specified by the environment variable.

3. **Overriding the Environment Variable**:

   If you want to override the environment variable, you can pass the `--threads` option explicitly:

   .. code-block:: bash

      python3 scripts/dev.py build cpp --threads 8

   In this case, the CLI will use 8 threads, ignoring the value set in `OSDK_DEV_CLI_THREADS`.

4. **Unsetting the Environment Variable**:

   To remove the environment variable, use the `unset` command:

   .. code-block:: bash

      unset OSDK_DEV_CLI_THREADS

----------------
Shell Completion
----------------

The Dev Script CLI supports tab-completion for commands, sub-commands, and
options in Bash, Zsh, Fish, and PowerShell.  Completion is powered by
Click's built-in shell completion machinery, so it is always up-to-date
with the live command tree — no regeneration needed when new commands are
added.

The CLI also detects when the shell is requesting completions and skips all
heavy initialisation (vcpkg probing, directory creation, etc.), so
tab-completion is fast.

^^^^
Bash
^^^^

Add the following line to ``~/.bashrc`` (or ``~/.bash_profile`` on macOS):

.. code-block:: bash

   eval "$(_DEV_COMPLETE=bash_source dev)"

Then reload your shell:

.. code-block:: bash

   source ~/.bashrc

^^^
Zsh
^^^

Add the following line to ``~/.zshrc``:

.. code-block:: zsh

   eval "$(_DEV_COMPLETE=zsh_source dev)"

Then reload your shell:

.. code-block:: zsh

   source ~/.zshrc

^^^^
Fish
^^^^

Add the following line to ``~/.config/fish/config.fish``:

.. code-block:: fish

   _DEV_COMPLETE=fish_source dev | source

Then reload your shell:

.. code-block:: fish

   source ~/.config/fish/config.fish

^^^^^^^^^^
PowerShell
^^^^^^^^^^

PowerShell requires a static completion script (Click has no built-in
PowerShell support).  Generate it once, then source it from your profile:

.. code-block:: powershell

   # Generate the script
   python scripts/dev.py completions generate --shell powershell --output-dir ~

   # Add to your $PROFILE (run once)
   Add-Content $PROFILE ". $HOME\dev_completions.ps1"

Re-run ``completions generate --shell powershell`` after adding new commands
to keep the PowerShell script up-to-date.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Using a different invocation name
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``eval`` snippets above assume ``dev`` is on your ``PATH``.  If you
invoke the script as ``python scripts/dev.py`` or via the ``dev.sh`` wrapper,
adjust the variable name to match:

.. code-block:: bash

   # For dev.py
   eval "$(_DEV_PY_COMPLETE=bash_source python scripts/dev.py)"

   # For dev.sh
   eval "$(_DEV_SH_COMPLETE=bash_source ./scripts/dev.sh)"

You can also generate a static file for any shell and inspect the
install hints with:

.. code-block:: bash

   python scripts/dev.py completions generate --shell bash
   python scripts/dev.py completions generate --shell zsh
   python scripts/dev.py completions generate --shell fish

Or print interactive setup instructions for your current shell:

.. code-block:: bash

   python scripts/dev.py completions setup

------------------------------------------
Writing New Plugins for the Dev Script Cli
------------------------------------------

The Cli is designed to be modular, allowing developers to easily add new functionality by creating plugins.
Plugins are Python scripts that define new commands or extend existing ones. Below is a guide to help you
write and integrate new plugins.

^^^^^^^^^^^^^^^^
Plugin Structure
^^^^^^^^^^^^^^^^

* File Naming Convention

  * The plugin file must be a Python file (.py).

  * Have a filename that starts with the prefix dev.
    Example: dev_example.py, dev_build_tools.py.

* The plugin file must be placed in one of the following directories.

  * Primary Location.

    * ouster-sdk/scripts/dev_script_library.
    * This is the main directory for the Ouster SDK Dev Script Cli plugins.
  * SDK Extensions Location.

    * sdk-extensions/scripts/dev_script_library.
    * Additional plugin modules can be placed here to extend the CLI.

* File Specifics

  * In the main body of the file, put any click groups/commands that you want to add.
  * At the bottom of the file, two functions are required

    * import_module: In the import_module function, register your command with the
      appropriate command group (e.g., build, test, lint, utils, or cleanup).
    * finalize: In the finalize function, if your plugin requires cleanup or additional
      setup after all plugins are loaded, implement the finalize function
      (If not needed, you still need to define the finalize function as an empty function).

* Example

.. code-block:: python

    import click
    import shutil
    import os


    @click.group()
    @click.pass_context()
    def my_util_group(ctx):
        pass


    @my_util_group.command()
    @click.pass_context()
    @click.option('--build-dir', default='build', help='Directory to clean.')
    @click.option('--artifacts-dir', default='artifacts', help='Artifacts directory to clean.')
    def clean(ctx, build_dir, artifacts_dir):
        """Clean build and artifacts directories."""
        for directory in [build_dir, artifacts_dir]:
            if os.path.exists(directory):
                print(f"Removing {directory}...")
                shutil.rmtree(directory)
            else:
                print(f"{directory} does not exist.")


    def import_module(click_context):
        click_context.top_level_group.add_command(my_util_group)


    def finalize(click_context):
        print("Finalizing plugin setup.")
