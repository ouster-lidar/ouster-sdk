ARG BASE="ubuntu:18.04"

FROM ${BASE}
ENV DEBIAN_FRONTEND=noninteractive \
    BUILD_HOME=/var/lib/build

# Install build dependencies
RUN set -xe \
&& apt-get update \
&& apt-get install -y --no-install-recommends \
 build-essential \
 cmake \
 libeigen3-dev \
 libjsoncpp-dev \
 libtins-dev \
 libpcap-dev \
 python3-dev \
 python3-pip \
 python3-venv \
 pybind11-dev \
# Install any additional available cpython versions for testing
 'python3.[6-9]-dev' \
 libglfw3-dev \
 libglew-dev \
 python3-breathe \
 doxygen \
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
&& python3 -m pip install --no-cache-dir --user -U pip tox

# Populate source dir
COPY . ${OUSTER_SDK_PATH}

# Entrypoint for running tox:
#
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
&& exec python3 -m tox -c ./src/python --workdir ${HOME}/tox \"$@\" \
", "tox-entrypoint"]
