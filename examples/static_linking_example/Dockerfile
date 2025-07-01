FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libtins-dev \
    libpcap-dev \
    libcurl4-openssl-dev \
    git \
    build-essential \
    cmake \
    zlib1g \
    zlib1g-dev \
    libglfw3-dev \
    libpng-dev \
    libflatbuffers-dev \
    libtbb-dev \
    robin-map-dev \
    libceres-dev

ENV WORKSPACE=/root
ENV INSTALL_DIR="/usr/local"
COPY . $WORKSPACE/
RUN cd $WORKSPACE && \
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF . && \
    cmake --build . --parallel 4 --target install

FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libtins-dev \
    libpcap-dev \
    libcurl4-openssl-dev \
    git \
    build-essential \
    cmake \
    libglfw3-dev \
    zlib1g \
    zlib1g-dev \
    libpng-dev \
    libflatbuffers-dev

ENV WORKSPACE=/root
ENV INSTALL_DIR="/usr/local"

COPY --from=0 $INSTALL_DIR $INSTALL_DIR
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/static_linking_example/CMakeLists.txt \
     examples/static_linking_example/main.cpp  $WORKSPACE/

RUN export CMAKE_PREFIX_PATH="$INSTALL_DIR" &&\
    mkdir -p $WORKSPACE/build &&\
    cd $WORKSPACE/build &&\
    cmake $WORKSPACE && cmake --build . --parallel 4

CMD $WORKSPACE/build/pcap_test /root/OS-2-32-U0_v2.0.0_1024x10.pcap
