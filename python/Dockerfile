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
# Install any additional available cpython versions for testing
 'python3.(7|8|9|10)-dev' \
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
    OUSTER_SDK_PATH="/opt/ouster_example"
WORKDIR ${BUILD_HOME}

RUN set -xe \
# use oldest available, supported python as tox default
&& PYTHON=$(which python3.7 python3.8 python3.9 python3.10 | head -1) \
&& $PYTHON -m pip install --no-cache-dir --user -U pip tox pybind11

# Populate source dir
COPY . ${OUSTER_SDK_PATH}

# Entrypoint for running tox:
#
# Usage: from OUSTER_SDK root, run
#   docker build . -f python/Dockerfile -t ouster-sdk-tox
#   docker run --rm -it [-e VAR=VAL ..] ouster-sdk-tox [TOX ARGS ..]
#
# Without any arguments: run unit tests with all available Python versions. See
# the tox.ini for other commands.
#
# The following environment variables can be set, which may be useful when
# running with additional host bind mounts:
# ARTIFACT_DIR: where to put test output. Defaults to ${BUILD_HOME}/artifacts
# WHEELS_DIR: where to look for wheels for running tests against wheels
# OUSTER_SDK_PATH: path of SDK source
#
ENTRYPOINT ["sh", "-c", "set -e \
&& rm -rf ./src && cp -a ${OUSTER_SDK_PATH} ./src \
&& export ARTIFACT_DIR=${ARTIFACT_DIR:-$BUILD_HOME/artifacts} \
&& . /etc/os-release && export ID VERSION_ID \
&& exec tox -c ./src/python --workdir ${HOME}/tox \"$@\" \
", "tox-entrypoint"]
