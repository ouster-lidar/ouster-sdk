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
 libpcap-dev \
 libtins-dev \
 libcurl4-openssl-dev \
 libglfw3-dev \
 libpng-dev \
 libflatbuffers-dev \
 gfortran \
# Python deps
 python3-dev \
 python3-pip \
 python3-venv \
 ccache \
 libxml2-utils \
 curl \
# Install any additional available cpython versions for testing
 'python3.(8|9|10)-dev' \
 wget \
&& rm -rf /var/lib/apt/lists

# Set up non-root build user and environment
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

RUN set -xe \
&& groupadd -g ${BUILD_GID} build \
&& useradd -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build \
# hack: allow the build user to install into /usr/local to avoid using venv for uv
&& chown -R ${BUILD_UID}:${BUILD_GID} /usr/local

USER build:build
ENV PATH="${PATH}:${BUILD_HOME}/.local/bin" \
    OUSTER_SDK_PATH="/opt/ouster-sdk"
WORKDIR ${BUILD_HOME}

# Install uv
RUN curl --proto '=https' --tlsv1.2 -LsSf https://github.com/astral-sh/uv/releases/download/0.8.19/uv-installer.sh | sh

# Populate source dir
COPY python/docker_install_reqs.sh python/setup.py python/requirements.txt ${OUSTER_SDK_PATH}/python/

RUN ${OUSTER_SDK_PATH}/python/docker_install_reqs.sh
