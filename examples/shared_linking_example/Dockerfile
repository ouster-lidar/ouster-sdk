# ============================== BASE CONTAINER ==============================
ARG BASE="ubuntu:22.04"
FROM ${BASE} AS BASE_IMAGE
ARG APT_PROXY=""
ENV WORKSPACE=/root

RUN /bin/sh -c "if ! [ -z \"$APT_PROXY\" ]; then \
        echo 'Using Proxy for APT'; \
        echo 'Acquire::http::Proxy \"$APT_PROXY\";' > /etc/apt/apt.conf.d/01proxy; \
    fi"

RUN apt-get update \
    && apt-get install -y python3 python3-venv python3-pip \
    && mkdir -p $WORKSPACE/scripts/dev_script_library

# ============================== BUILD STAGE ==============================
FROM BASE_IMAGE AS BUILD
ARG VCPKG_BINARY_SOURCES=""
ENV VCPKG_BINARY_SOURCES=$VCPKG_BINARY_SOURCES

COPY ./scripts/dev.py ./scripts/requirements.txt \
    $WORKSPACE/scripts/
COPY ./scripts/dev.py ./scripts/requirements.txt \
    ./scripts/dev_script_library/build_libs.py \
    ./scripts/dev_script_library/context.py \
    ./scripts/dev_script_library/dev_dependencies.py \
    $WORKSPACE/scripts/dev_script_library/

RUN python3 -m venv $WORKSPACE/venv && \
    . $WORKSPACE/venv/bin/activate && \
    cd $WORKSPACE && \
    python3 ./scripts/dev.py utils install-vcpkg-package-requirements && \
    python3 ./scripts/dev.py utils enable-local-vcpkg

ENV INSTALL_DIR="/usr/local"

COPY . $WORKSPACE/

RUN cd $WORKSPACE && \
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DBUILD_EXAMPLES=OFF \
        -DBUILD_SHARED_LIBRARY=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
        -DCMAKE_TOOLCHAIN_FILE=$WORKSPACE/.osdkv2/vcpkg/scripts/buildsystems/vcpkg.cmake . &&\
    cmake --build . --parallel 4 --target install

# ============================== FINAL STAGE ==============================
FROM BASE_IMAGE

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libflatbuffers-dev \
    && rm -rf /var/lib/apt/lists/*

ENV WORKSPACE=/root
ENV INSTALL_DIR="/usr/local"

COPY --from=BUILD $INSTALL_DIR $INSTALL_DIR
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/shared_linking_example/CMakeLists.txt \
     examples/shared_linking_example/main.cpp  $WORKSPACE/

RUN export CMAKE_PREFIX_PATH="$INSTALL_DIR" &&\
    mkdir -p $WORKSPACE/build &&\
    cd $WORKSPACE/build &&\
    cmake $WORKSPACE && cmake --build . --parallel 4

CMD $WORKSPACE/build/pcap_test /root/OS-2-32-U0_v2.0.0_1024x10.pcap
