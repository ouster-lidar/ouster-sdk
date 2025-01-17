#! /bin/bash
set -e
# Variables
BUILD_DIR=$(mktemp -d)
SOURCE_DIR=$(cd $(dirname $0) && pwd)
CURRENT_DIR=$PWD
CLANG_TIDY_THREADS=${CLANG_TIDY_THREADS:-$(nproc)}

# Check For Tools
which clang-tidy 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: clang-tidy needs to be installed to run this tool"
    exit 1
else
    clang-tidy --help | grep "config-file" 2>&1 > /dev/null
    if [ $? -ne 0 ];
    then
        echo "ERROR: clang-tidy needs to be updated, missing --config-file cli support"
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

# Register an on exit cleanup function to delete the temporary
# directory.
cleanup() {      
  rm -rf "$BUILD_DIR"
}
trap cleanup EXIT

# Map out the files to be checked, if no files are specified on the command line
# include all of the source files
FILES="$SOURCE_DIR/ouster_client $SOURCE_DIR/ouster_pcap $SOURCE_DIR/ouster_osf $SOURCE_DIR/ouster_viz $SOURCE_DIR/python $SOURCE_DIR/examples"
if [ "$#" -ne 0 ]; then
    FILES=""
    for item in "$@"
    do
        # Grab the realpath of the files before we change directories
        FILES="$FILES $(realpath $item)"
    done
fi

# Enter the temporary build directory
pushd $BUILD_DIR > /dev/null

# Generate the compile_commands.json file for the tidy step
cmake $SOURCE_DIR -DBUILD_EXAMPLES=ON -DBUILD_PCAP=ON -DBUILD_OSF=ON -DBUILD_VIZ=ON -DBUILD_PYTHON_MODULE=ON -DSKIP_SDK_FIND=ON -DBUILD_TESTING=ON -DPYTHON_EXECUTABLE=$(which python3)
cmake --build . --target ouster_generate_header cpp_gen

# Run tidy
python3 $SOURCE_DIR/_clang-tidy.py --clang-tidy-bin $(which clang-tidy) --compile-commands-json $PWD/compile_commands.json --clang-tidy-config $SOURCE_DIR/.clang-tidy -j $CLANG_TIDY_THREADS --build-dir $PWD --json-output $CURRENT_DIR/clang-tidy-output.json --raw-output $CURRENT_DIR/clang-tidy-output.txt --paths $FILES --json-summary-output  $CURRENT_DIR/clang-tidy-output-summary.json

popd > /dev/null