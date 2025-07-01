#! /bin/bash
set -e
set +x
# Variables
BUILD_DIR=$(mktemp -d)
# Register an on exit cleanup function to delete the temporary
# directory.
cleanup() {
  rm -rf "$BUILD_DIR"
}
trap cleanup EXIT

SCRIPT_DIR=$(cd $(dirname $0) && pwd)
SOURCE_DIR=$(cd $SCRIPT_DIR/.. && pwd)
CURRENT_DIR=$PWD
FB_INFER_THREADS=${FB_INFER_THREADS:-$(nproc)}
FB_INFER_VERSION="v1.2.0"
FB_INFER_LOCATION="https://github.com/facebook/infer/releases/tag/$FB_INFER_VERSION"
VCPKG_MANIFEST_MODE=${VCPKG_MANIFEST_MODE:-}
VCPKG_ROOT=${VCPKG_ROOT:-}

if ! [ -z "$VCPKG_MANIFEST_MODE" ];
then
    VCPKG_MANIFEST_MODE="-DVCPKG_MANIFEST_MODE=$VCPKG_MANIFEST_MODE"
fi

if ! [ -z "$VCPKG_ROOT" ];
then
    VCPKG_TOOLCHAIN="-DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
fi
# Check For Tools
set +e
which infer 2>&1 > /dev/null
if [ $? -ne 0 ];
then
    echo "ERROR: infer needs to be installed to run this tool"
    echo "Please refer to $FB_INFER_LOCATION"
    exit 1
else
    ACTUAL_INFER_VERSION=$(infer --version | grep "Infer version" | sed -rE 's/Infer version (v[\d]*\.*[\d]*\.*[\d]*)/\1/')
    if [ "$FB_INFER_VERSION" != "$ACTUAL_INFER_VERSION" ];
    then
        echo "WARNING: The version of infer you have is not recommended"
        echo "Actual:      \"$ACTUAL_INFER_VERSION\""
        echo "Recommended: \"$FB_INFER_VERSION\""
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

set -e

# Enter the temporary build directory
pushd $BUILD_DIR > /dev/null

# Generate the compile_commands.json file for the tidy step
cmake $SOURCE_DIR -B $BUILD_DIR -DBUILD_EXAMPLES=OFF -DBUILD_PCAP=ON -DBUILD_OSF=ON -DBUILD_VIZ=ON -DBUILD_PYTHON_MODULE=ON -DSKIP_SDK_FIND=ON -DBUILD_TESTING=ON -DPYTHON_EXECUTABLE=$(which python3) $VCPKG_TOOLCHAIN $VCPKG_MANIFEST_MODE
cmake --build $BUILD_DIR --target ouster_generate_header cpp_gen

# Run the analysis
infer --compilation-database compile_commands.json --cost --bufferoverrun --racerd --dump-duplicate-symbols --loop-hoisting --pulse --topl --siof --starvation -j $FB_INFER_THREADS
rm -rf $CURRENT_DIR/infer-out
cp -r infer-out $CURRENT_DIR/
popd > /dev/null
