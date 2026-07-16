:tocdepth: 2
.. _dev-scripts:

Ouster Developer Scripts
========================

The **Developer Scripts** library (``scripts/dev_script_library/``) is the public,
cross-platform developer toolchain for the Ouster SDK.  It is invoked through
the thin wrapper ``scripts/dev.py`` (or ``scripts/dev.sh`` on POSIX systems)
and exposes a structured `Click <https://click.palletsprojects.com/>`_ CLI
organised into the command groups described below.

.. note::

   Every command supports the ``--help`` flag.  Run
   ``python scripts/dev.py <group> <command> --help`` for the full option
   listing of any individual command.

   Environment variables follow the pattern
   ``OSDK_DEV_CLI_<OPTION_NAME_UPPER_SNAKE>`` and can be used in place of any
   CLI flag (e.g. ``OSDK_DEV_CLI_THREADS=8``).

----

Prerequisites
-------------

Install the Python dependencies declared in ``scripts/requirements.txt``
before using any Dev Script command:

.. code-block:: console

   $ python3 -m pip install -r scripts/requirements.txt

The ``dev.py`` entry-point will also auto-install missing dependencies on
first run, but only when a virtual environment is active.  Outside a venv it
will print an error and exit rather than modifying the system Python.

----

Entry Point
-----------

.. code-block:: console

   # Linux / macOS
   $ ./scripts/dev.sh <group> <command> [OPTIONS]

   # Windows (PowerShell or Command Prompt)
   > scripts\dev.bat <group> <command> [OPTIONS]

   # Cross-platform (all platforms)
   $ python scripts/dev.py <group> <command> [OPTIONS]

Top-level ``--help``:

.. code-block:: console

   $ python scripts/dev.py --help

----

.. _dev-scripts-shell-completion:

Shell Completion
----------------

The Dev Script CLI supports tab-completion for commands, sub-commands, and
options.  Completion is powered by Click's built-in machinery and calls back
into ``dev.py`` at completion time, so it is always in sync with the live
command tree — no regeneration needed when new commands are added.

The CLI detects completion callbacks early and skips all heavy initialisation
(vcpkg probing, directory creation, etc.), so tab-completion is fast.

Bash
~~~~

Add to ``~/.bashrc`` (or ``~/.bash_profile`` on macOS):

.. code-block:: bash

   eval "$(_DEV_COMPLETE=bash_source <Path to dev.sh>)"

Then reload:

.. code-block:: console

   $ source ~/.bashrc

Zsh
~~~

Add to ``~/.zshrc``:

.. code-block:: zsh

   eval "$(_DEV_COMPLETE=zsh_source <Path to dev.sh>)"

Then reload:

.. code-block:: console

   $ source ~/.zshrc

Fish
~~~~

Add to ``~/.config/fish/config.fish``:

.. code-block:: fish

   _DEV_COMPLETE=fish_source <Path to dev.sh> | source

Then reload:

.. code-block:: console

   $ source ~/.config/fish/config.fish

PowerShell
~~~~~~~~~~

PowerShell requires a static script (Click has no built-in PowerShell
support).  Generate it once and source it from your profile:

.. code-block:: powershell

   # Generate the script
   python scripts/dev.py completions generate --shell powershell --output-dir ~

   # Add to your $PROFILE (run once)
   Add-Content $PROFILE ". $HOME\dev_completions.ps1"

Re-run ``completions generate --shell powershell`` after adding new commands.

Adjusting the invocation name
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``eval`` snippets above assume a ``dev`` binary is on your ``PATH``.  If
you invoke the script directly, adjust the variable name to match:

.. code-block:: bash

   # Invoked as: python scripts/dev.py
   eval "$(_DEV_PY_COMPLETE=bash_source python scripts/dev.py)"

   # Invoked as: ./scripts/dev.sh
   eval "$(_DEV_SH_COMPLETE=bash_source ./scripts/dev.sh)"

Interactive setup instructions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``completions setup`` command prints the correct one-liner for your
current shell:

.. code-block:: console

   $ python scripts/dev.py completions setup

To generate static completion files for distribution:

.. code-block:: console

   $ python scripts/dev.py completions generate --shell bash
   $ python scripts/dev.py completions generate --shell zsh
   $ python scripts/dev.py completions generate --shell fish
   $ python scripts/dev.py completions generate          # all shells

----

Command Groups
--------------

build
~~~~~

Commands that compile SDK components or generate SDK documentation.

.. _cmd-build-cpp:

build cpp
^^^^^^^^^

Build the C++ SDK using CMake and vcpkg.

.. code-block:: console

   $ python scripts/dev.py build cpp [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--package``
     - ``False``
     - Enable shared-library (package) build.
   * - ``--cmake-bin TEXT``
     - ``cmake``
     - Path to an alternative CMake binary.
   * - ``--vcpkg-toolchain TEXT``
     - *auto-detected*
     - Path to the vcpkg toolchain file.
   * - ``--vcpkg-triplet TEXT``
     - *auto-detected*
     - vcpkg triplet (e.g. ``x64-linux``, ``x64-windows-static-md``).
   * - ``--use-system-libs``
     - ``False``
     - Use system libraries instead of vcpkg.
   * - ``--threads INT``
     - ``cpu_count / 2``
     - Parallel build threads.
   * - ``--build-type [Release|Debug|RelWithDebInfo]``
     - ``Release``
     - CMake build type.
   * - ``--no-manifest-mode``
     - ``False``
     - Disable vcpkg manifest mode.
   * - ``--no-examples``
     - ``False``
     - Skip building examples.
   * - ``--no-tests``
     - ``False``
     - Skip building tests.
   * - ``--install-dir PATH``
     - *artifacts dir*
     - Installation prefix for the built SDK.
   * - ``--cmake-arg TEXT``
     - *(multiple)*
     - Extra tokens forwarded to the CMake configure step.
   * - ``--profile-build``
     - ``False``
     - Enable build profiling (requires ``clang`` and ``ClangBuildAnalyzer``).
   * - ``--coverage-flags``
     - ``False``
     - Instrument the build for code-coverage (Linux only).

**Examples**

.. code-block:: console

   # Standard release build
   $ python scripts/dev.py build cpp

   # Debug build with system libraries
   $ python scripts/dev.py build cpp --build-type Debug --use-system-libs

   # Package build (shared library + CPack archive)
   $ python scripts/dev.py build cpp --package --install-dir /tmp/sdk-install

   # Pass extra CMake flags
   $ python scripts/dev.py build cpp --cmake-arg -DBUILD_VIZ=OFF

.. _cmd-build-python:

build python
^^^^^^^^^^^^

Build the Python SDK wheel or install it in editable mode.

.. code-block:: console

   $ python scripts/dev.py build python [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--output-type [editable|wheel]``
     - ``editable``
     - Build mode: install in-place or produce a ``.whl`` file.
   * - ``--vcpkg-toolchain TEXT``
     - *auto-detected*
     - Path to the vcpkg toolchain file.
   * - ``--vcpkg-triplet TEXT``
     - *auto-detected*
     - vcpkg triplet.
   * - ``--use-system-libs``
     - ``False``
     - Use system libraries instead of vcpkg.
   * - ``--threads INT``
     - ``cpu_count / 2``
     - Parallel build threads.
   * - ``--build-type [Release|Debug|RelWithDebInfo]``
     - ``Release``
     - CMake build type.
   * - ``--no-manifest-mode``
     - ``False``
     - Disable vcpkg manifest mode.
   * - ``--profile-build``
     - ``False``
     - Enable build profiling.
   * - ``--coverage-flags``
     - ``False``
     - Enable coverage instrumentation (Linux only).
   * - ``--no-clean``
     - ``False``
     - Preserve build directories after building.

**Examples**

.. code-block:: console

   # Editable install (fastest for development)
   $ python scripts/dev.py build python

   # Produce a distributable wheel
   $ python scripts/dev.py build python --output-type wheel

.. _cmd-build-compile-commands:

build compile-commands
^^^^^^^^^^^^^^^^^^^^^^

Generate ``compile_commands.json`` for IDE integration (clangd, etc.).

.. code-block:: console

   $ python scripts/dev.py build compile-commands [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--output PATH``
     - ``<sdk_root>/compile_commands.json``
     - Destination path for the generated file.
   * - ``--cmake-bin TEXT``
     - ``cmake``
     - Path to an alternative CMake binary.
   * - ``--vcpkg-toolchain TEXT``
     - *auto-detected*
     - Path to the vcpkg toolchain file.
   * - ``--vcpkg-triplet TEXT``
     - *auto-detected*
     - vcpkg triplet.
   * - ``--use-system-libs``
     - ``False``
     - Use system libraries instead of vcpkg.
   * - ``--no-manifest-mode``
     - ``False``
     - Disable vcpkg manifest mode.
   * - ``--threads INT``
     - ``cpu_count / 2``
     - Parallel build threads.

.. _cmd-build-doxygen:

build doxygen
^^^^^^^^^^^^^

Generate Doxygen HTML documentation and check for undocumented parameters.

.. code-block:: console

   $ python scripts/dev.py build doxygen [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--doxygen-bin TEXT``
     - ``doxygen``
     - Path to an alternative ``doxygen`` binary.
   * - ``--output-dir PATH``
     - *docs build dir*
     - Doxygen output directory.
   * - ``--param-log PATH``
     - *(none)*
     - Write parameter-direction warnings to this file.
   * - ``--no-werror``
     - ``False``
     - Do not fail on Doxygen warnings.
   * - ``--warning-log PATH``
     - *output-dir/warning_log.log*
     - Path for the Doxygen warning log.
   * - ``--no-inline-param-check``
     - ``False``
     - Skip the inline ``[in]``/``[out]`` direction check.

.. note::

   The CI-pinned Doxygen version is **1.11.0**.  Using a different version
   may produce different output.

.. warning::

   This command is **not supported on Windows**.  Running it on Windows will
   exit immediately with the error ``not supported on windows yet``.

**Examples**

.. code-block:: console

   $ python scripts/dev.py build doxygen
   $ python scripts/dev.py build doxygen --no-werror --output-dir /tmp/doxy

.. _cmd-build-docs:

build docs
^^^^^^^^^^

Generate the full SDK documentation site (Sphinx + Doxygen) for a given
version.  This command is provided by the ``sdk-extensions`` plugin
(``dev_docs_sdkx.py``) and is only available when the ``sdk-extensions``
submodule is present.

.. code-block:: console

   $ python scripts/dev.py build docs [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--version TEXT``
     - ``latest``
     - Version label to embed in the generated docs (e.g. ``latest``, ``0.16.0``).
   * - ``--html-destination-dir PATH``
     - ``artifacts``
     - Directory where the generated HTML is written.
   * - ``--no-cpp-api / --cpp-api``
     - ``False``
     - Skip C++ API generation (Doxygen, Exhale, Breathe).
   * - ``--no-python-api / --python-api``
     - ``False``
     - Skip Python API generation (``sphinx-apidoc``).
   * - ``--no-python-build / --python-build``
     - ``False``
     - Skip ``requirements.txt`` install and Python SDK source build.

.. warning::

   This command is **not supported on Windows**.  Running it on Windows will
   exit immediately with the error ``not supported on windows yet``.

**Examples**

.. code-block:: console

   $ python scripts/dev.py build docs --version latest --html-destination-dir artifacts
   $ python scripts/dev.py build docs --version 0.16.0 --no-python-build

----

test
~~~~

Commands that execute test suites.

.. _cmd-test-cpp:

test cpp
^^^^^^^^

Run the C++ test suite via ``ctest``.

.. code-block:: console

   $ python scripts/dev.py test cpp [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--threads INT``
     - ``cpu_count / 2``
     - Parallel test threads passed to ``ctest -j``.
   * - ``--test-dir-override PATH``
     - *cmake build dir*
     - Run ``ctest`` from this directory instead of the default build dir.
   * - ``--use-shared-libs``
     - ``False``
     - Run tests against the shared-library build.
   * - ``--shared-libs-path PATH``
     - *(none)*
     - Path to the shared libraries used when ``--use-shared-libs`` is set.
   * - ``--ctest-bin TEXT``
     - ``ctest``
     - Path to an alternative ``ctest`` binary.
   * - ``--build-type [Release|Debug]``
     - ``Release``
     - Build configuration to test.

**Examples**

.. code-block:: console

   $ python scripts/dev.py test cpp
   $ python scripts/dev.py test cpp --threads 4 --build-type Debug

.. _cmd-test-python:

test python
^^^^^^^^^^^

Run the Python test suite via ``pytest``.

.. code-block:: console

   $ python scripts/dev.py test python [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--test-data-dir PATH``
     - *env: TEST_DATA_DIR*
     - Directory containing test data for integration tests.
   * - ``--threads INT``
     - ``cpu_count / 2``
     - Parallel pytest workers (``pytest-xdist``).
   * - ``--skip-integration-tests``
     - ``False``
     - Skip integration tests even when ``--test-data-dir`` is provided.

**Examples**

.. code-block:: console

   $ python scripts/dev.py test python
   $ python scripts/dev.py test python --test-data-dir /data/sdk-test-data

.. _cmd-test-self-tests:

test self-tests
^^^^^^^^^^^^^^^

Run the Dev Script library's own unit tests (``scripts/dev_script_tests/``).

.. code-block:: console

   $ python scripts/dev.py test self-tests [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--threads INT``
     - ``cpu_count / 2``
     - Parallel pytest workers.

**Examples**

.. code-block:: console

   $ python scripts/dev.py test self-tests
   $ python scripts/dev.py test self-tests --threads 1

.. _cmd-test-docs:

test docs
^^^^^^^^^

Run documentation tests (CLI pytest and optional C++/Python snippet tests).
This command is provided by the ``sdk-extensions`` plugin (``dev_docs_tests.py``)
and is only available when the ``sdk-extensions`` submodule is present.

.. code-block:: console

   $ python scripts/dev.py test docs [OPTIONS]

**Examples**

.. code-block:: console

   $ python scripts/dev.py test docs --no-cpp-api --no-python-api

----

lint
~~~~

Commands that enforce code quality.

.. _cmd-lint-flake8:

lint flake8
^^^^^^^^^^^

Run the ``flake8`` Python linter over the SDK source tree.

.. code-block:: console

   $ python scripts/dev.py lint flake8 [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--config PATH``
     - ``python/.flake8``
     - Alternative flake8 configuration file.
   * - ``--output-file PATH``
     - *(stdout)*
     - Write results in JUnit-XML format to this file.

.. _cmd-lint-mypy:

lint mypy
^^^^^^^^^

Run the ``mypy`` static type checker over the SDK source tree.

.. code-block:: console

   $ python scripts/dev.py lint mypy [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--output-file PATH``
     - *(stdout)*
     - Write results in JUnit-XML format to this file.

.. _cmd-lint-mypy-stubs:

lint mypy-stubs
^^^^^^^^^^^^^^^

Validate the ``mypy`` stub file for the C++ bindings module.

.. code-block:: console

   $ python scripts/dev.py lint mypy-stubs

.. _cmd-lint-clang-format:

lint clang-format
^^^^^^^^^^^^^^^^^

Run ``clang-format`` over all tracked C/C++ source files.

.. code-block:: console

   $ python scripts/dev.py lint clang-format [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--clang-format-bin TEXT``
     - ``clang-format``
     - Path to an alternative ``clang-format`` binary.
   * - ``--fix``
     - ``False``
     - Apply automatic formatting fixes (prompts for confirmation).
   * - ``--threads INT``
     - *(all cores)*
     - Parallel formatting threads.

.. _cmd-lint-clang-tidy:

lint clang-tidy
^^^^^^^^^^^^^^^

Run ``clang-tidy`` (or ``clangd-tidy``) static analysis over C++ sources.

.. code-block:: console

   $ python scripts/dev.py lint clang-tidy [OPTIONS] [PATHS]...

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--clang-tidy-bin TEXT``
     - *auto-detected*
     - Path to ``clang-tidy`` or ``clangd-tidy``.
   * - ``--clang-apply-replacements-bin TEXT``
     - ``clang-apply-replacements``
     - Used when ``--fix`` is enabled.
   * - ``--fix``
     - ``False``
     - Apply automatic fixes (requires confirmation).
   * - ``--raw-output PATH``
     - *build dir*
     - Write raw tidy output to this file.
   * - ``--json-output PATH``
     - *build dir*
     - Write structured JSON output to this file.
   * - ``--json-summary-output PATH``
     - *build dir*
     - Write a summary JSON (issue category counts) to this file.
   * - ``--timing-json PATH``
     - *build dir*
     - Write per-check timing data (``clang-tidy`` only).
   * - ``--quiet``
     - ``False``
     - Suppress stdout diagnostic messages.
   * - ``--cmake-bin TEXT``
     - ``cmake``
     - Path to an alternative CMake binary.
   * - ``--vcpkg-toolchain TEXT``
     - *auto-detected*
     - Path to the vcpkg toolchain file.
   * - ``--vcpkg-triplet TEXT``
     - *auto-detected*
     - vcpkg triplet.
   * - ``--use-system-libs``
     - ``False``
     - Use system libraries instead of vcpkg.
   * - ``--threads INT``
     - ``cpu_count / 2``
     - Parallel analysis threads.
   * - ``--no-manifest-mode``
     - ``False``
     - Disable vcpkg manifest mode.
   * - ``--diff-against TEXT``
     - *(none)*
     - Git ref (e.g. ``origin/develop``) — only report warnings on lines
       changed since this ref.  Useful as a checkin gate.
   * - ``PATHS``
     - *all ouster_\* dirs*
     - One or more files or directories to analyse.

**Examples**

.. code-block:: console

   # Analyse the whole SDK
   $ python scripts/dev.py lint clang-tidy

   # Checkin gate: only fail on new warnings relative to develop
   $ python scripts/dev.py lint clang-tidy --diff-against origin/develop

   # Analyse a single file
   $ python scripts/dev.py lint clang-tidy ouster_core/src/types.cpp

.. _cmd-lint-check-cpp-exports:

lint check-cpp-exports
^^^^^^^^^^^^^^^^^^^^^^

Verify that every public C++ symbol in the shared-library headers carries the
correct ``OUSTER_API_CLASS`` / ``OUSTER_API_FUNCTION`` annotation.

.. code-block:: console

   $ python scripts/dev.py lint check-cpp-exports [OPTIONS] [PATHS]...

**Options** (same vcpkg / cmake flags as ``lint clang-tidy`` plus):

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--verbose``
     - ``False``
     - Print each processed file.
   * - ``PATHS``
     - *all public include dirs*
     - Header directories or files to check.

.. _cmd-lint-infer-cpp-static-analysis:

lint infer-cpp-static-analysis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run Facebook `Infer <https://fbinfer.com/>`_ static analysis over the C++
sources.

.. code-block:: console

   $ python scripts/dev.py lint infer-cpp-static-analysis [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--infer-bin TEXT``
     - ``infer``
     - Path to the ``infer`` binary.
   * - ``--output-dir PATH``
     - *artifacts dir*
     - Directory for the ``infer-out`` results folder.
   * - ``--cmake-bin TEXT``
     - ``cmake``
     - Path to an alternative CMake binary.
   * - ``--vcpkg-toolchain TEXT``
     - *auto-detected*
     - Path to the vcpkg toolchain file.
   * - ``--vcpkg-triplet TEXT``
     - *auto-detected*
     - vcpkg triplet.
   * - ``--use-system-libs``
     - ``False``
     - Use system libraries instead of vcpkg.
   * - ``--threads INT``
     - ``cpu_count``
     - Parallel analysis threads.
   * - ``--no-manifest-mode``
     - ``False``
     - Disable vcpkg manifest mode.

.. _cmd-lint-python-ignores:

lint python-ignores
^^^^^^^^^^^^^^^^^^^

Audit ``# type: ignore`` and ``# noqa`` suppression comments in the Python
source tree and emit a JSON report.

.. code-block:: console

   $ python scripts/dev.py lint python-ignores [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--quiet``
     - ``False``
     - Suppress JSON output to stdout.
   * - ``--output-dir PATH``
     - *cwd*
     - Directory for the output file.
   * - ``--output-file TEXT``
     - ``mypy_ignores.json``
     - Output filename.

----

cleanup
~~~~~~~

Commands that remove generated artefacts.

.. _cmd-cleanup-build:

cleanup build
^^^^^^^^^^^^^

Remove the CMake and Python build directories.

.. code-block:: console

   $ python scripts/dev.py cleanup build

.. _cmd-cleanup-artifacts:

cleanup artifacts
^^^^^^^^^^^^^^^^^

Remove the SDK artifacts directory.

.. code-block:: console

   $ python scripts/dev.py cleanup artifacts

.. _cmd-cleanup-all:

cleanup all
^^^^^^^^^^^

Remove **all** directories created by the Dev Script context: build,
artifacts, and the persistent vcpkg directory (``~/.osdkv2`` by default).

.. code-block:: console

   $ python scripts/dev.py cleanup all

----

utils
~~~~~

Utility commands for setting up the development environment.

.. _cmd-utils-enable-local-vcpkg:

utils enable-local-vcpkg
^^^^^^^^^^^^^^^^^^^^^^^^

Clone and bootstrap a local vcpkg repository under ``~/.osdkv2/vcpkg``.

.. code-block:: console

   $ python scripts/dev.py utils enable-local-vcpkg [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--reinit``
     - ``False``
     - Delete the existing vcpkg directory and re-clone from scratch.

.. _cmd-utils-install-system-packages:

utils install-system-packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install the system-level C++ build dependencies using the platform package
manager (``apt-get`` on Debian/Ubuntu, ``brew`` on macOS).

.. code-block:: console

   $ python scripts/dev.py utils install-system-packages [OPTIONS]

**Options**

.. list-table::
   :header-rows: 1
   :widths: 30 10 60

   * - Option
     - Default
     - Description
   * - ``--doxygen``
     - ``False``
     - Also install Doxygen 1.11.0.
   * - ``--clangformat``
     - ``False``
     - Also install ``clang-format``.

.. _cmd-utils-install-vcpkg-package-requirements:

utils install-vcpkg-package-requirements
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install the OS-level packages required by vcpkg itself (e.g. ``flex``,
``bison``, ``libxinerama-dev`` on Ubuntu).

.. code-block:: console

   $ python scripts/dev.py utils install-vcpkg-package-requirements

.. note::

   This command is a no-op on Windows; vcpkg has no additional system
   dependencies there.

----

Library Internals
-----------------

The following modules are consumed by the commands above.  They are not
intended to be called directly by end-users but are documented here for
contributors.

``build_libs.py``
~~~~~~~~~~~~~~~~~

Core helper classes shared by all commands:

* **RunCommand** – thin wrapper around ``subprocess.Popen`` with timing,
  TTY-passthrough, and error-propagation support.
* **Python** – manages a ``venv`` and delegates to ``uv pip`` for fast,
  reproducible dependency installation.
* **CMake** – drives CMake configure / build / install / package steps.
* **Doxygen** – generates and validates Doxygen output from a template
  ``Doxyfile``.
* **ClangThrowMap** – walks a C++ AST to map functions to the exception types
  they throw.

Standalone helper functions:

* ``check_for_python_lib(lib)`` – assert a Python package is importable.
* ``check_for_tool(tool)`` – assert a binary is on ``PATH``.
* ``parse_version(sdk_path)`` – read the SDK version from ``CMakeLists.txt``.
* ``initialize_vcpkg(vcpkg_dir)`` – clone / update the vcpkg repository.
* ``generate_compile_commands(...)`` – produce ``compile_commands.json``.
* ``perf_json_combine(jsons, output)`` – merge Chrome-trace JSON files.
* ``get_env_from_sourced_shell(script)`` – capture the environment exported
  by a shell script.

``context.py``
~~~~~~~~~~~~~~

**ClickContext** is the shared state object passed through the Click context
(``ctx.obj``) to every command.  It exposes:

* Computed paths: ``sdk_dir``, ``cmake_build_dir``, ``docs_build_dir``,
  ``python_build_dir``, ``sdk_artifact_dir``, ``vcpkg_dir``.
* **BuildOptions** – centralises vcpkg / CMake argument processing and
  auto-detects the correct vcpkg triplet for the current platform.
* ``add_module(file)`` – dynamically load a Dev Script plugin module.
* ``finalize()`` – call ``finalize()`` on all loaded plugins.

----

Environment Variables
---------------------

The following environment variables influence Dev Script behaviour:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Variable
     - Description
   * - ``DEV_PERSISTENT_DIR``
     - Override the default persistent directory (``~/.osdkv2``).
   * - ``VCPKG_ROOT``
     - Fallback vcpkg root when ``--vcpkg-toolchain`` is not set.
   * - ``VCPKG_BINARY_SOURCES``
     - Binary caching source string forwarded to CMake.
   * - ``TEST_DATA_DIR``
     - Default test-data directory for ``test python``.
   * - ``CCACHE_DIR``
     - When set and ``ccache`` is on ``PATH``, enables compiler caching.
   * - ``OSDK_DEV_CLI_<OPTION>``
     - Any CLI option can be set via this pattern
       (e.g. ``OSDK_DEV_CLI_THREADS=8``).

----

Running the Self-Tests
----------------------

The Dev Script library ships its own test suite under
``scripts/dev_script_tests/``.  Run it with:

.. code-block:: console

   $ python scripts/dev.py test self-tests

Or directly with pytest:

.. code-block:: console

   $ python -m pytest scripts/dev_script_tests/ -v

----

Adding a New Command
--------------------

1. Create a new module ``scripts/dev_script_library/dev_<feature>.py``.
2. Define your Click commands and implement ``import_module(click_context)``
   to register them on the appropriate group
   (``click_context.build_group``, ``click_context.lint_group``, etc.).
3. Implement a no-op ``finalize(click_context)`` function.
4. Register the module in ``scripts/dev.py`` via ``ctx.add_module(...)``.
5. Add corresponding tests under ``scripts/dev_script_tests/``.
6. Document the new command in this file.
