import os
import shutil
import subprocess
import sys
import time
import shlex
from flufl.lock import Lock
from flufl.lock import LockState
from datetime import timedelta
from string import Template
import re
import glob
import json
import signal


def os_independent_shlex_split(path):
    if sys.platform == 'win32':
        return shlex.split(path, posix=0)
    else:
        return shlex.split(path)


def venv_folder_template(python_bin, venv_base):
    python_bin = os.path.basename(python_bin)
    python_bin = python_bin.replace(' ', '_')
    return os.path.join(venv_base, f"venv-{python_bin}")


class Defaults:
    default_artifact_dir = os.path.join(os.getcwd(), 'artifacts')
    default_build_dir = os.path.join(os.getcwd(), 'build')
    default_root_dir = os.path.join(os.path.dirname(__file__), "..", "..")
    default_build_type = "Release"
    default_build_generator = "Ninja"


class IndividualTimer:
    def __init__(self, label):
        self._start = time.time()
        self._stop = None
        self._label = label

    def start(self):
        self._start = time.time()

    def stop(self):
        self._stop = time.time()

    def get_duration(self):
        if self._stop is not None:
            return self._stop - self._start
        else:
            raise Exception("Need call stop on the timer before duration can be calculated")

    def __str__(self):
        total = self.get_duration()
        min = int(total / 60)
        sec = int(total % 60)
        return f"{self._label}: {min} minutes {sec} seconds"


class BuildTimer:
    def __init__(self):
        self._timers = {}

    def make_timer(self, label):
        if label not in self._timers:
            self._timers[label] = IndividualTimer(label)
            return self._timers[label]
        else:
            return self.make_timer(f"{label}{time.time()}")

    def __str__(self):
        return "\n".join([str(self._timers[x]) for x in self._timers])


class RunCommand:
    def __init__(self, tty=False):
        self._timers = BuildTimer()
        self._tty = tty

    def run_command(self, *args, cwd=os.getcwd(), env=None, throw_on_error=True,
                    tty=None, quiet=False):
        output = ""
        timer = self._timers.make_timer(f"{args}")
        timer.start()
        if env is None:
            env = os.environ.copy()
        use_tty = self._tty
        if tty is not None:
            use_tty = tty
        try:
            print(f"Running: {args} In Directory: {cwd}")

            process = subprocess.Popen(args,
                                       env=env,
                                       stdout=sys.stdout if use_tty else subprocess.PIPE,
                                       stderr=subprocess.STDOUT if use_tty else subprocess.PIPE,
                                       cwd=os.path.normpath(cwd),
                                       text=True,
                                       bufsize=1)
            try:
                stdout, stderr = process.communicate()
            except KeyboardInterrupt:
                process.send_signal(signal.SIGINT)
                raise
            if not use_tty:
                if stdout:
                    print(stdout)
                output = stdout or ""

            if process.returncode != 0:
                if not use_tty and stderr:
                    print("STDERR:", stderr)
                if throw_on_error:
                    raise Exception(f"Error Running Process {args}, "
                                    f"Return Code: {process.returncode}")
        finally:
            timer.stop()

        return output

    def get_timers(self):
        return self._timers


class Python(RunCommand):
    def __init__(self, python_bin, venv_dir, replace_bin_with_venv=True, make_venv=True,
                 pip_version=None, pip_lock_file=None, tty=False):
        RunCommand.__init__(self, tty=tty)
        self._python_bin = python_bin
        self._env = dict(os.environ)
        self._venv_dir = venv_dir
        self._pip_version = pip_version
        self._pip_lock_file = pip_lock_file
        self._pip_folder_lock = None
        if self._pip_lock_file is not None:
            self._pip_folder_lock = Lock(self._pip_lock_file)

        if make_venv:
            self.make_venv()
        if replace_bin_with_venv:
            if sys.platform == 'win32':
                self._python_venv_bin = os.path.join(self._venv_dir, "Scripts")
            else:
                self._python_venv_bin = os.path.join(self._venv_dir, "bin")
            self._env["PATH"] = self._python_venv_bin + os.pathsep + self._env["PATH"]
            self._python_venv_bin = os.path.join(self._python_venv_bin, "python")
            self._python_bin = self._python_venv_bin
        self.ensure_uv_installed()

    def make_venv(self):
        """ Make a virtual environment """
        if not os.path.exists(self._venv_dir):
            self.run_python_command("-m", "venv", "--clear", self._venv_dir, env=self._env)

    def install_uv_if_not_exists(self):
        self.lock()
        try:
            try:
                self.run_python_command("-m", "pip", "--version", env=self._env)
            except Exception:
                raise RuntimeError("Failed to find pip. Install uv manually.")
            try:
                self.run_python_command("-m", "pip", "install", "--upgrade",
                                        "uv", env=self._env)
            except Exception:
                raise RuntimeError("""Failed to install uv with pip.
                                Please install uv manually.
                                https://docs.astral.sh/uv/getting-started/installation/#installing-uv""")
        finally:
            self.unlock()

    def ensure_uv_installed(self):
        try:
            self.show_uv_version()
        except Exception:
            print("uv not found; attempting installation with pip…")
            self.install_uv_if_not_exists()

    def show_uv_version(self):
        self.run_python_command("-m", "uv", "pip", "show", "uv", env=self._env)

    def upgrade_uv(self):
        self.run_python_command("-m", "uv", "pip", "install",
                                "--upgrade", "uv",
                                env=self._env)
        self.run_python_command("-m", "uv", "pip", "show",
                                "uv",
                                env=self._env)

    def install_pip_deps(self, *deps, folder=None):
        print("Installing pip dependencies")
        args = ["-m", "uv", "pip", "install", "--only-binary", ":all:"]
        if folder is not None:
            args.extend(["--upgrade", "--pre", "--no-index", "-f", folder])
        args.extend(deps)

        result = self.run_python_command(*args)
        return result

    def install_pip_deps_from_requirements(self, requirements_file):
        print("Installing pip requirements")

        args = ["-m", "uv", "pip", "install", "-r", os.path.join(requirements_file)]

        result = self.run_python_command(*args)
        return result

    def run_python_command(self, *args, env=None, tty=None):
        print(f"Running Python Command: {args}")
        if env is None:
            env = self._env
        run_args = []
        run_args.extend(
            os_independent_shlex_split(self._python_bin))
        run_args.extend(args)
        return self.run_command(*run_args, env=env, tty=tty)

    def lock(self):
        if self._pip_folder_lock is not None and not self.have_lock():
            print("Locking pip")
            self._pip_folder_lock.lock()
            self._pip_folder_lock.lifetime = timedelta(seconds=(5 * 50))

    def unlock(self):
        if self._pip_folder_lock is not None:
            if self.have_lock():
                print("Unlocking pip")
                self._pip_folder_lock.unlock()

    def have_lock(self):
        return self.is_locked() and self._pip_folder_lock.state == LockState.ours

    def is_locked(self):
        return self._pip_folder_lock is not None and self._pip_folder_lock.is_locked


class CMake(RunCommand):
    def __init__(self, source_dir, build_dir, artifact_dir, cmake_args=[],
                 build_type=Defaults.default_build_type,
                 cmake_path="cmake",
                 cmake_generate_prepend=None,
                 build_generator=Defaults.default_build_generator,
                 env=None,
                 tty=False):
        RunCommand.__init__(self, tty)
        self._source_dir = source_dir
        self._build_dir = build_dir
        self._cmake_args = cmake_args
        self._build_type = build_type
        self._cmake_path = cmake_path
        self._build_generator = build_generator
        self._artifact_dir = artifact_dir
        self._cmake_generate_prepend = cmake_generate_prepend
        if env is None:
            env = os.environ.copy()
        self._env = env

        # Process ccache
        if "CCACHE_DIR" in os.environ:
            if shutil.which("ccache") is not None:
                print("Class CMake FOUND CCACHE")
                self._cmake_args.append("-DCMAKE_CXX_COMPILER_LAUNCHER=ccache")

    def clear(self):
        if os.path.exists(self._build_dir):
            shutil.rmtree(self._build_dir, ignore_errors=True)
        if not os.path.exists(self._build_dir):
            os.makedirs(self._build_dir)

    def generate(self):
        print(self.run_cmake_command("-B", self._build_dir, "-S", self._source_dir,
                                     *self._cmake_args,
                                     f"-DCMAKE_BUILD_TYPE={self._build_type}",
                                     cwd=self._build_dir,
                                     tty=self._tty,
                                     env=self._env,
                                     generate=True))

    def build(self, targets=None, threads=os.cpu_count()):
        args = ["--build", self._build_dir,
                "--config", f"{self._build_type}",
                f"-j{threads}"]
        if targets is not None:
            args.append("--target")
            args.extend(targets)
        print(self.run_cmake_command(*args,
                                     cwd=self._build_dir,
                                     tty=self._tty,
                                     env=self._env))
        print(str(self.get_timers()))

    def install(self, prefix=None):
        prefix = self._artifact_dir if prefix is None else prefix
        print(self.run_cmake_command("--install", self._build_dir,
                                     "--config", f"{self._build_type}",
                                     "--prefix", prefix,
                                     cwd=self._build_dir,
                                     tty=self._tty,
                                     env=self._env))

    def make_package(self, prefix=None):
        print(self.run_cmake_command("--build", self._build_dir,
                                     "--config", f"{self._build_type}",
                                     "--target", "package",
                                     cwd=self._build_dir,
                                     tty=self._tty,
                                     env=self._env))
        paths_to_copy = [os.path.join(self._build_dir, "**/ouster*-sdk-*.tgz"),
                         os.path.join(self._build_dir, "**/ouster*-sdk-*.tar.gz"),
                         os.path.join(self._build_dir, "**/ouster*-sdk-*.zip")]
        for path in paths_to_copy:
            for item in glob.glob(path, recursive=True):
                print(f"Copying {item} to {self._artifact_dir}")
                shutil.copy(item, self._artifact_dir)
                if prefix is not None:
                    print(f"Copying {item} to {prefix}")
                    shutil.copy(item, prefix)
        return paths_to_copy

    def get_cmake_version(self):
        return self.run_cmake_command("--version", tty=self._tty,
                                     env=self._env)

    def run_cmake_command(self, *args, cwd=os.getcwd(), env=None, tty=None, generate=False):
        run_args = []
        if generate and self._cmake_generate_prepend is not None:
            run_args.append(self._cmake_generate_prepend)
        run_args.extend(
            os_independent_shlex_split(self._cmake_path))
        run_args.extend(args)

        return self.run_command(*run_args, cwd=cwd, env=env, tty=tty)


class Doxygen(RunCommand):
    def __init__(self, project_name, version, output_dir,
                 source_dir, enhanced_warnings=False, tty=False,
                 doxygen_bin="doxygen", warning_log=None):
        RunCommand.__init__(self, tty)

        self._project_name = project_name
        self._version = version
        self._output_dir = output_dir
        self._source_dir = source_dir
        self._doxy_file_out = os.path.join(output_dir, "Doxyfile")
        if warning_log is not None:
            self._log_file_out = warning_log
        else:
            self._log_file_out = os.path.join(output_dir, "warning_log.log")
        self._doxy_working_dir = os.path.join(self._source_dir, "docs")
        self._doxy_template_file = os.path.join(self._doxy_working_dir, "Doxyfile")
        self._enhanced_warnings = enhanced_warnings
        self._doxygen_bin = doxygen_bin

    def _generate_doxygen_config(self):
        dictionary = {
            'project': self._project_name,
            'version': self._version,
            'output_dir': self._output_dir,
            'enhanced_warnings': "YES" if self._enhanced_warnings else "NO",
            'warn_log_file': self._log_file_out,
        }

        with open(self._doxy_template_file, 'r') as template_file:
            template = Template(template_file.read())
            with open(self._doxy_file_out, 'w') as template_file_out:
                template_file_out.write(template.substitute(dictionary))

    def generate_doxygen(self):
        self._generate_doxygen_config()
        args = [self._doxygen_bin, self._doxy_file_out]
        self.run_command(*args, cwd=self._doxy_working_dir)

    def get_warnings(self):
        warnings = []
        if os.path.exists(self._log_file_out):
            with open(self._log_file_out, 'r') as log_file:
                for item in log_file.read().split('\n'):
                    if len(item) > 0:
                        found = False
                        for check in ["operator=", "operator<=", "operator>=",
                                      "operator==", "operator!=", "operator<",
                                      "operator>", "operator[]", "operator++",
                                      "operator--", "operator()", "operator->",
                                      "operator!=", "operator+"]:
                            if check in item:
                                found = True
                        if not found:
                            warnings.append(item)
            return warnings
        else:
            print("ERROR: Doxygen log file not found, please generate doxygen first")
            return None


class Clang:
    def __init__(self):
        import clang.cindex
        self._clang = clang.cindex

    def _is_functional(self, node):
        return node.kind == self._clang.CursorKind.CXX_METHOD or \
                 node.kind == self._clang.CursorKind.FUNCTION_DECL or \
                 node.kind == self._clang.CursorKind.CONSTRUCTOR or \
                 node.kind == self._clang.CursorKind.DESTRUCTOR or \
                 node.kind == self._clang.CursorKind.FUNCTION_TEMPLATE

    def _is_throw(self, node):
        return node.kind == self._clang.CursorKind.CXX_THROW_EXPR

    def _is_exceptionable_type(self, node):
        return node.kind == self._clang.CursorKind.INTEGER_LITERAL or \
                 node.kind == self._clang.CursorKind.FLOATING_LITERAL or \
                 node.kind == self._clang.CursorKind.IMAGINARY_LITERAL or \
                 node.kind == self._clang.CursorKind.STRING_LITERAL or \
                 node.kind == self._clang.CursorKind.CHARACTER_LITERAL or \
                 node.kind == self._clang.CursorKind.TYPE_REF

    def _find_containing_function(self, node):
        # Traverse up the AST to find the containing function
        while node:
            if self._is_functional(node):
                return node
            node = node.prev
        return None

    def _find_throw_type(self, node):
        # Traverse down the AST to find the type of the exception
        for item in self._iter(node):
            if self._is_exceptionable_type(item):
                return item.type.spelling

    def _iter(self, root):
        root.prev = None
        queue = [root]

        # Breadth first traversal of all of the nodes
        while len(queue) > 0:
            current_node = queue.pop(0)
            yield current_node
            for child in current_node.get_children():
                child.prev = current_node
                queue.append(child)

    def _get_full_name(self, node):
        stack = [node.displayname]
        filename = node.location.file
        current_node = node.lexical_parent
        while current_node is not None and current_node.spelling != filename:
            stack.append(current_node.spelling)
            current_node = current_node.lexical_parent

        return "::".join(reversed(stack[:-1]))


class ClangThrowMap(Clang):
    def __init__(self, dirs=None, files=None, progress_bar=False):
        Clang.__init__(self)

        def tqdm(iterable, *args, **kwargs):
            return iterable
        self._tqdm = tqdm
        if progress_bar:
            try:
                import tqdm
                self._tqdm = tqdm.tqdm
            except ImportError:
                print("tqdm not found, cant display progress bar. please install tqdm")
                progress_bar = False

        self._progress_bar = progress_bar
        self._dirs = dirs
        self._files = files
        self._throw_map = {}

        files_to_process = []
        if dirs is not None:
            for item in dirs:
                for sub_item in glob.glob(os.path.join(item, "**", "*.cpp"), recursive=True):
                    files_to_process.append(sub_item)
                for sub_item in glob.glob(os.path.join(item, "**", "*.h"), recursive=True):
                    files_to_process.append(sub_item)
        if files is not None:
            for item in files:
                files_to_process.append(item)

        for item in self._tqdm(files_to_process):
            self._generate_throw_map_for_file(item)

    def _process_throw(self, node):
        # Process the throw expression to extract relevant information
        function_node = self._get_full_name(self._find_containing_function(node))
        throw_type = self._find_throw_type(node)
        self._throw_map[function_node] = throw_type

    def _generate_throw_map_for_file(self, file_path):
        index = self._clang.Index.create()
        tu = index.parse(file_path)
        for current_node in self._iter(tu.cursor):
            if self._is_throw(current_node) and \
               file_path == str(current_node.location.file):
                self._process_throw(current_node)

    def get_throw_map(self):
        return self._throw_map


def check_for_python_lib(python_lib, fail_on_missing=True, display_name=None):
    """ Check if the python library is installed """
    try:
        import importlib
        importlib.import_module(python_lib)
        return True
    except ImportError:
        if display_name is not None:
            python_lib = display_name
        print(f"ERROR: {python_lib} (via pip) to be installed to run this tool")
        print(f"ERROR: run python3 -m uv pip install {python_lib}")
        if fail_on_missing:
            sys.exit(1)
        return False


def check_for_tool(tool, fail_on_missing=True, tool_path=None):
    """ Check if the tool is installed """
    import shutil
    if tool_path and os.path.exists(tool_path):
        return tool_path
    found = shutil.which(tool)
    if found:
        return found
    if fail_on_missing:
        raise RuntimeError(f"""ERROR: {tool} needs to be installed to run this tool
                           ERROR: Install {tool} OR run ./scripts/dev.sh utils install-vcpkg-package-requirements""")
    return None


def confirm_auto_fix():
    user_input = ""
    while user_input not in ["y", "n"]:
        print("The fix option will make changes to the local code repository.")
        user_input = input("Are you sure? [Y/N]? ").lower()
    return user_input == "y"


def parse_version(sdk_path):
    with open(os.path.join(sdk_path, 'CMakeLists.txt')) as listfile:
        content = listfile.read()
        groups = re.search(r"set\(OusterSDK_VERSION_STRING ([^-\)]+)(-(.*))?\)", content)
        return groups.group(1) + (groups.group(3) or "")


def initialize_vcpkg(vcpkg_dir, reinit=False):
    if reinit:
        shutil.rmtree(vcpkg_dir, ignore_errors=True)

    if not os.path.exists(vcpkg_dir) or reinit:
        import git
        print("Initializing build toolchains")
        git.Repo.clone_from('https://github.com/microsoft/vcpkg.git', vcpkg_dir)
        args = []
        if os.name == 'nt':
            args.append(os.path.join(vcpkg_dir, "bootstrap-vcpkg.bat"))
        else:
            args.append(os.path.join(vcpkg_dir, "bootstrap-vcpkg.sh"))
        subprocess.check_output(args, cwd=vcpkg_dir)
    else:
        import git
        print("Updating git repository for build toolchains")
        # Initialize the repo object pointing to existing dir
        repo = git.Repo(vcpkg_dir)
        # Pull the latest changes from the origin remote
        repo.remotes.origin.fetch()
        repo.remotes.origin.pull()


def perf_json_combine(perf_jsons, output_path):
    """ Combine chrome trace json files """
    all_events = []
    for perf_json in perf_jsons:
        with open(perf_json, 'r') as f:
            try:
                data = json.load(f)
                all_events.extend(data['traceEvents'])
            except json.JSONDecodeError:
                print(f"WARNING: Failed to decode JSON from {perf_json}")

    with open(output_path, 'w') as f:
        json.dump({'traceEvents': all_events}, f)


def get_env_from_sourced_shell(shell_script):
    """ Get the environment variables from a sourced shell script """
    assert os.path.exists(shell_script), f"Shell script {shell_script} does not exist"
    args = []
    if os.name == 'nt':
        args.append("cmd")
        args.append("/c")
        args.append(f"call {shell_script} > NUL && "
                    "python -c \"import os; import json; print(json.dumps(dict(os.environ)))\"")
    else:
        args.append("bash")
        args.append("-c")
        args.append(f"source {shell_script} &> /dev/null && "
                    "python3 -c 'import os; import json; print(json.dumps(dict(os.environ)))'")
    output = subprocess.check_output(args, text=True)
    return json.loads(output)


def generate_compile_commands(output, sdk_dir, artifact_dir, build_dir,
                              toolchain=None, triplet=None,
                              manifest_mode=True, cmake_path="cmake", env=None):
    manifest_mode_text = "ON" if manifest_mode else "OFF"
    cmake_args = ["-DBUILD_TESTING=ON",
                  "-DBUILD_PYTHON_MODULE=ON",
                  "-DSKIP_SDK_FIND=ON",
                  "-DUSE_OPENMP=OFF",
                  f"-DPYTHON_EXECUTABLE={sys.executable}",
                  "-DBUILD_EXAMPLES=ON"]

    if toolchain is not None:
        cmake_args.append(f"-DCMAKE_TOOLCHAIN_FILE={toolchain}")
        cmake_args.append(f"-DVCPKG_MANIFEST_MODE={manifest_mode_text}")
    if triplet is not None:
        cmake_args.append(f"-DVCPKG_TARGET_TRIPLET={triplet}")

    compile_commands_dir = os.path.join(build_dir,
                                        "compile_commands")
    os.makedirs(compile_commands_dir, exist_ok=True)
    print(f"Generating compile commands in {compile_commands_dir}, Args: {cmake_args}")
    cmake = CMake(sdk_dir,
                  compile_commands_dir,
                  artifact_dir,
                  cmake_args=cmake_args,
                  cmake_path=cmake_path,
                  env=env,
                  tty=True)
    cmake.generate()
    cmake.build(targets=["ouster_generate_header", "cpp_gen"])
    compile_commands_json = os.path.join(compile_commands_dir, "**",
                                         "compile_commands.json")
    compile_commands_json = glob.glob(compile_commands_json,
                                      recursive=True)[0]
    shutil.copy(compile_commands_json, output)
