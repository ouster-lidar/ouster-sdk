#! /bin/bash
set -e
# Variables
BUILD_DIR=$(mktemp -d)
SCRIPT_DIR=$(cd $(dirname $0) && pwd)
SOURCE_DIR=$(cd $SCRIPT_DIR/.. && pwd)
CURRENT_DIR=$PWD
VCPKG_MANIFEST_MODE=${VCPKG_MANIFEST_MODE:-}
CLANG_TIDY_THREADS=${CLANG_TIDY_THREADS:-$(nproc)}
CLANG_TIDY_BIN=${CLANG_TIDY_BIN:-clang-tidy}
CLANG_APPLY_BIN=${CLANG_APPLY_BIN:-clang-apply-replacements}

if ! [ -z "$VCPKG_MANIFEST_MODE" ];
then
    VCPKG_MANIFEST_MODE="-DVCPKG_MANIFEST_MODE=$VCPKG_MANIFEST_MODE"
fi

set +e
# Check For Tools
which $CLANG_TIDY_BIN 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: clang-tidy needs to be installed to run this tool"
    exit 1
else
    MAJOR_VERSION=$($CLANG_TIDY_BIN --version 2>&1 | head -n 1 | awk '{print $4}' | awk -F '.' '{print $1}')
    if [ $MAJOR_VERSION -lt 16 ];
    then
        echo "ERROR: clang-tidy needs to be updated, minimum version is 16"
        echo "       current clang-tidy version is $MAJOR_VERSION"
        exit 2
    fi
fi

# Check For Tools
which $CLANG_APPLY_BIN 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: cant find clang-apply-replacements"
    exit 1
else
    MAJOR_VERSION=$($CLANG_APPLY_BIN --version 2>&1 | awk '{print $3}' | awk -F '.' '{print $1}')
    if [ $MAJOR_VERSION -lt 16 ];
    then
        echo "ERROR: clang-apply-replacements needs to be updated, minimum version is 16"
        echo "       current clang-apply-replacements version is $MAJOR_VERSION"
        exit 2
    fi

fi

which python3 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: python3 needs to be installed to run this tool"
    exit 3
fi

which cmake 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: cmake needs to be installed to run this tool"
    exit 4
fi

python3 -c "import pybind11" 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: pybind11 (via pip) to be installed to run this tool"
    echo "ERROR: run python3 -m pip install pybind11"
    exit 5
fi
python3 -c "import tqdm" 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: tqdm (via pip) to be installed to run this tool"
    echo "ERROR: run python3 -m pip install tqdm"
    exit 6
fi

set -e

# Register an on exit cleanup function to delete the temporary
# directory.
cleanup() {
  rm -rf "$BUILD_DIR"
}
trap cleanup EXIT

args=()
fix=""
config_file="$SOURCE_DIR/.clang-tidy"
for item in "$@"
do
    if [ "$item" != "--fix" ];
    then
        args+=("$item")
    else
        if [ -z "$fix" ];
        then
            echo "The fix option will make changes to the local code repository."
            read -p "Are you sure? [Y/N]" -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]
            then
                fix="--fix";
                config_file="${config_file}-autofix"
            else
                echo "Canceling Fixes"
                exit 1
            fi
        fi
    fi
done

# Map out the files to be checked, if no files are specified on the command line
# include all of the source files
FILES="$SOURCE_DIR/ouster_client $SOURCE_DIR/ouster_pcap $SOURCE_DIR/ouster_osf $SOURCE_DIR/ouster_viz $SOURCE_DIR/python $SOURCE_DIR/examples"
if [ "${#args[@]}" -ne 0 ]; then
    FILES=""
    for item in "${args[@]}"
    do
        # Grab the realpath of the files before we change directories
        FILES="$FILES $(realpath $item)"
    done
fi

# Enter the temporary build directory
pushd $BUILD_DIR > /dev/null

# Generate the compile_commands.json file for the tidy step
cmake $SOURCE_DIR -DBUILD_EXAMPLES=ON -DBUILD_PCAP=ON -DBUILD_OSF=ON -DBUILD_VIZ=ON -DBUILD_PYTHON_MODULE=ON -DSKIP_SDK_FIND=ON -DBUILD_TESTING=ON -DPYTHON_EXECUTABLE=$(which python3) $VCPKG_MANIFEST_MODE
cmake --build . --target ouster_generate_header cpp_gen

# Run tidy python script
python3 $SCRIPT_DIR/_clang-tidy.py --clang-tidy-bin $CLANG_TIDY_BIN --clang-apply-replacement-bin $CLANG_APPLY_BIN --compile-commands-json $PWD/compile_commands.json --clang-tidy-config $config_file -j $CLANG_TIDY_THREADS --build-dir $PWD --json-output $CURRENT_DIR/clang-tidy-output.json --raw-output $CURRENT_DIR/clang-tidy-output.txt --paths $FILES --json-summary-output  $CURRENT_DIR/clang-tidy-output-summary.json --timing-json $CURRENT_DIR/clang-tidy-output-timing.json $fix

popd > /dev/null
