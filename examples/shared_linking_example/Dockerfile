FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    git build-essential cmake \
    curl zip unzip tar pkg-config \
    python3 libxinerama-dev libxcursor-dev \
    xorg-dev libglu1-mesa-dev flex bison \
    libeigen3-dev libceres-dev

ENV WORKSPACE=/root
ENV INSTALL_DIR="/usr/local"
RUN mkdir -p /opt/vcpkg && cd /opt && git clone https://github.com/microsoft/vcpkg.git \
    && cd vcpkg && ./bootstrap-vcpkg.sh && ./vcpkg install "curl[core]" libtins \
    glfw3 "glad[gl-api-33]" libpng flatbuffers zlib gtest openssl tbb robin-map

COPY . $WORKSPACE/
RUN cd $WORKSPACE && \
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DBUILD_EXAMPLES=OFF -DVCPKG_MANIFEST_MODE=OFF \
        -DBUILD_SHARED_LIBRARY=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake . &&\
    cmake --build . --parallel 4 --target install

FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libflatbuffers-dev \
    git \
    build-essential \
    cmake

ENV WORKSPACE=/root
ENV INSTALL_DIR="/usr/local"

COPY --from=0 $INSTALL_DIR $INSTALL_DIR
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/shared_linking_example/CMakeLists.txt \
     examples/shared_linking_example/main.cpp  $WORKSPACE/

RUN export CMAKE_PREFIX_PATH="$INSTALL_DIR" &&\
    mkdir -p $WORKSPACE/build &&\
    cd $WORKSPACE/build &&\
    cmake $WORKSPACE && cmake --build . --parallel 4

CMD $WORKSPACE/build/pcap_test /root/OS-2-32-U0_v2.0.0_1024x10.pcap
