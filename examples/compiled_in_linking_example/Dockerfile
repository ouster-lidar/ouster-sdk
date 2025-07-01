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
COPY . $WORKSPACE/sdk/
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/compiled_in_linking_example/CMakeLists.txt \
     examples/compiled_in_linking_example/main.cpp  $WORKSPACE/

RUN cd $WORKSPACE && \
    cmake -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF . && \
    cmake --build . --parallel 4

CMD $WORKSPACE/pcap_test /root/OS-2-32-U0_v2.0.0_1024x10.pcap
