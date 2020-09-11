FROM ubuntu:18.04

RUN sed -ie 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.ocf.berkeley.edu/ubuntu/|' /etc/apt/sources.list
RUN ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime && DEBIAN_FRONTEND=noninteractive && apt-get update && \
        apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        libeigen3-dev \
        libjsoncpp-dev \
        libpcap-dev \
        libpth-dev \
        libglfw3-dev \
        libglew-dev \
        python3-pytest \
        python3-psutil \
        libtclap-dev \
        python3-setuptools \
        python3-wheel \
        python3-dev \
        python3-pip \
        python3-numpy \
        libtins-dev \
        curl && \
    pip3 install --upgrade setuptools &&\
    curl -o pybind11.deb "http://ftp.us.debian.org/debian/pool/main/p/pybind11/pybind11-dev_2.2.4-2_all.deb" &&\
    echo ca3c556f95a6e77bc727c2d168985477ae26f91210bf33e4fc87aa156b560ba7  pybind11.deb | sha256sum -c &&\
    dpkg -i pybind11.deb &&\
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /var/tmp/* /tmp/*

COPY CMakeLists.txt /shared_sw/CMakeLists.txt
WORKDIR /shared_sw/build
CMD cmake .. && make -j$(nproc)
