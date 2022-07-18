ARG BASE="ubuntu:18.04"
ARG VCPKG_REF=2022.02.23

FROM ${BASE}
ENV DEBIAN_FRONTEND=noninteractive \
    BUILD_HOME=/var/lib/build

# Set up non-root build user and environment
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

# Install build dependencies
RUN set -xe \
&& apt-get update \
&& apt-get install -y --no-install-recommends \
 build-essential \
 cmake \
 libeigen3-dev \
 libjsoncpp-dev \
 libpcap-dev \
 python3-dev \
 python3-pip \
 python3-venv \
 pybind11-dev \
# Install any additional available cpython versions for testing
 'python3.[7-9]-dev' \
 libglfw3-dev \
 libglew-dev \
 python3-breathe \
 doxygen \
 git \
 ninja-build zip curl unzip bison flex wget \
&& rm -rf /var/lib/apt/lists \
&& groupadd build -r -g ${BUILD_GID} \
&& useradd build -rm -u ${BUILD_UID} -g build -d /opt/ws -s /bin/bash \
&& mkdir /opt/vcpkg && chown build:build /opt/vcpkg \
&& wget https://github.com/seladb/PcapPlusPlus/archive/refs/tags/v22.05.tar.gz \
&& echo "8ffe33406b903fe104f7bf0cf7254028bb3a9997bb5f4b226276af3b2a310e1ed1621558b9fd96ed09c75fa05dfcbfbd7111fe4b19a9288eb214ebcf1254cf6b  v22.05.tar.gz" | sha512sum --check \ 
&& tar -zxf v22.05.tar.gz \
&& ls -a \
&& rm -f v22.05.tar.gz \
&& cd ./PcapPlusPlus-22.05/ \
&& ./configure-linux.sh --default \
&& make && make install \
&& rm -rf ./PcapPlusPlus-22.05

USER build:build
ENV PATH="${PATH}:${BUILD_HOME}/.local/bin" \
    OUSTER_SDK_PATH="/opt/ouster_example"
WORKDIR ${BUILD_HOME}

RUN set -xe \
&& python3.7 -m pip install --no-cache-dir --user -U pip tox

# Populate source dir
COPY . ${OUSTER_SDK_PATH}

# Entrypoint for running tox:
#
# Build from OUSTER_SDK root: docker build -t ouster-sdk-tox -f python/Dockerfile
# Usage: docker run --rm -it [-e VAR=VAL ..] ouster-sdk-tox [TOX ARGS ..]
#
# Without any arguments: run unit tests with all available Python versions. See
# the tox.ini for other commands.
#
# The following environment variables can be set, which may be useful when
# running with additional host bind mounts:
# ARTIFACT_DIR: where to put test output. Defaults to ${BUILD_HOME}/artifacts
# WHEELS_DIR: where to look for wheels for running tests against wheels
# OUSTER_SDK_PTH: path of SDK source
#
ENTRYPOINT ["sh", "-c", "set -e \
&& rm -rf ./src && cp -a ${OUSTER_SDK_PATH} ./src \
&& export ARTIFACT_DIR=${ARTIFACT_DIR:-$BUILD_HOME/artifacts} \
&& . /etc/os-release && export ID VERSION_ID \
&& exec python3.7 -m tox -c ./src/python --workdir ${HOME}/tox \"$@\" \
", "tox-entrypoint"]
