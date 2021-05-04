ARG ROS_DISTRO=melodic

FROM ros:${ROS_DISTRO}-ros-core
ENV DEBIAN_FRONTEND=noninteractive \
    BUILD_HOME=/var/lib/build \
    OUSTER_SDK_PATH="/opt/ouster_example"

RUN set -xe \
# Turn off installing extra packages globally to slim down rosdep install
&& echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/01norecommend \
&& apt-get update \
&& apt-get install -y \
 build-essential \
 cmake \
# Rosdep needs a bit of coaxing to install...
 python3-rosdep python3-rospkg python3-catkin-pkg

# Install build dependencies using rosdep
COPY ouster_ros/package.xml ${OUSTER_SDK_PATH}/ouster_ros/package.xml

RUN set -xe \
&& apt-get update \
&& rosdep init \
&& rosdep update --rosdistro=${ROS_DISTRO} \
&& rosdep install -y --from-paths ${OUSTER_SDK_PATH}

# Set up non-root build user
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

RUN set -xe \
&& groupadd -o -g ${BUILD_GID} build \
&& useradd -o -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build

USER build:build

# Populate source dir
COPY cmake ${OUSTER_SDK_PATH}/cmake
COPY ouster_client ${OUSTER_SDK_PATH}/ouster_client
COPY ouster_viz ${OUSTER_SDK_PATH}/ouster_viz
COPY ouster_ros ${OUSTER_SDK_PATH}/ouster_ros

# Build ouster_ros
WORKDIR ${BUILD_HOME}

RUN set -xe \
&& mkdir -p ./ws/src \
&& ln -s ${OUSTER_SDK_PATH} ./ws/src \
&& /opt/ros/${ROS_DISTRO}/env.sh catkin_make -C ./ws -DCMAKE_BUILD_TYPE=Release

# Entrypoint for running Ouster ros:
#
# Usage: docker run --rm -it ouster-ros [COMMAND ..]
#
# Without any arguments: run a shell in the build environment. Any arguments are
# interpreted as a command to run instead.
#
ENTRYPOINT ["sh", "-c", "set -e \
&& . ./ws/devel/setup.sh \
&& exec \"$@\" \
", "ros-entrypoint"]

CMD ["/bin/bash"]
