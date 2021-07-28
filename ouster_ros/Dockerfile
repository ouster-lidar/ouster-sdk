ARG ROS_DISTRO=melodic

FROM ros:${ROS_DISTRO}-ros-core AS build-env
ENV DEBIAN_FRONTEND=noninteractive \
    BUILD_HOME=/var/lib/build \
    OUSTER_SDK_PATH=/opt/ouster_example

RUN set -xue \
# Kinetic and melodic have python3 packages but they seem to conflict
&& [ $ROS_DISTRO = "noetic" ] && PY=python3 || PY=python \
# Turn off installing extra packages globally to slim down rosdep install
&& echo 'APT::Install-Recommends "0";' > /etc/apt/apt.conf.d/01norecommend \
&& apt-get update \
&& apt-get install -y \
 build-essential cmake \
 fakeroot dpkg-dev debhelper \
 $PY-rosdep $PY-rospkg $PY-bloom

# Set up non-root build user
ARG BUILD_UID=1000
ARG BUILD_GID=${BUILD_UID}

RUN set -xe \
&& groupadd -o -g ${BUILD_GID} build \
&& useradd -o -u ${BUILD_UID} -d ${BUILD_HOME} -rm -s /bin/bash -g build build

# Install build dependencies using rosdep
COPY --chown=build:build ouster_ros/package.xml ${OUSTER_SDK_PATH}/ouster_ros/package.xml

RUN set -xe \
&& apt-get update \
&& rosdep init \
&& rosdep update --rosdistro=${ROS_DISTRO} \
&& rosdep install -y --from-paths ${OUSTER_SDK_PATH}

# Set up build environment
COPY --chown=build:build . ${OUSTER_SDK_PATH}

USER build:build
WORKDIR ${BUILD_HOME}

RUN set -xe \
&& mkdir src \
&& ln -s ${OUSTER_SDK_PATH} ./src


FROM build-env

RUN /opt/ros/${ROS_DISTRO}/env.sh catkin_make -DCMAKE_BUILD_TYPE=Release

# Entrypoint for running Ouster ros:
#
# Usage: docker run --rm -it ouster-ros [ouster.launch parameters ..]
#
ENTRYPOINT ["bash", "-c", "set -e \
&& . ./devel/setup.bash \
&& roslaunch ouster_ros ouster.launch \"$@\" \
", "ros-entrypoint"]
