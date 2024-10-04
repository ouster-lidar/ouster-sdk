ARG BASE="ubuntu:20.04"

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
 doxygen \
# SDK deps \
 libeigen3-dev \
 libjsoncpp-dev \
 libpcap-dev \
 libtins-dev \
 libcurl4-openssl-dev \
 libglfw3-dev \
 libglew-dev \
 libspdlog-dev \
 libpng-dev \
 libflatbuffers-dev \
# Python deps
 python3-dev \
 python3-pip \
 python3-venv \
 ccache \
 libxml2-utils \
# Install any additional available cpython versions for testing
 'python3.(8|9|10)-dev' \
 wget \
&& rm -rf /var/lib/apt/lists

# Set up non-root build user and environment
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

RUN set -xe \
&& groupadd -g ${BUILD_GID} build \
&& useradd -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build

USER build:build
ENV PATH="${PATH}:${BUILD_HOME}/.local/bin" \
    OUSTER_SDK_PATH="/opt/ouster_sdk"
WORKDIR ${BUILD_HOME}

# Populate source dir
COPY python/docker_install_reqs.sh python/setup.py ${OUSTER_SDK_PATH}/python/
COPY python/src/ouster/sdk/mapping/requirements.json ${OUSTER_SDK_PATH}/python/src/ouster/sdk/mapping/

RUN ${OUSTER_SDK_PATH}/python/docker_install_reqs.sh
