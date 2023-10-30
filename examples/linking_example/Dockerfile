FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libjsoncpp-dev \
    libtins-dev \
    libpcap-dev \
    libcurl4-openssl-dev \
    git \
    build-essential \
    cmake \
    libspdlog-dev

ENV WORKSPACE=/root
ENV INSTALL_DIR="/usr/local"
COPY . $WORKSPACE/
RUN cd $WORKSPACE && \
    cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DBUILD_PCAP=ON -DBUILD_OSF=OFF -DBUILD_VIZ=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF . && \
    cmake --build . --target install

FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libjsoncpp-dev \
    libtins-dev \
    libpcap-dev \
    libcurl4-openssl-dev \
    git \
    build-essential \
    cmake \
    libspdlog-dev

ENV WORKSPACE=/root
ENV INSTALL_DIR="/usr/local"

COPY --from=0 $INSTALL_DIR $INSTALL_DIR
COPY tests/pcaps/OS-2-32-U0_v2.0.0_1024x10.pcap examples/linking_example/CMakeLists.txt \
     examples/linking_example/main.cpp  $WORKSPACE/

RUN export CMAKE_PREFIX_PATH="$INSTALL_DIR" &&\
    mkdir -p $WORKSPACE/build &&\
    cd $WORKSPACE/build &&\
    cmake $WORKSPACE && cmake --build .

CMD $WORKSPACE/build/pcap_test /root/OS-2-32-U0_v2.0.0_1024x10.pcap
