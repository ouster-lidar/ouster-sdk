FROM ros:melodic-perception-bionic

ENV DEBIAN_FRONTEND noninteractive
ENV TERM xterm

# Some packages copied from:
# https://gitlab.com/nvidia/opengl/blob/ubuntu16.04/base/Dockerfile

RUN apt-get update \
  && apt-get dist-upgrade -y \
  && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    dialog \
    make \
    gcc \
    g++ \
    locales \
    wget \
    software-properties-common \
    sudo \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxau6 \
    libxdmcp6 \
    libxcb1 \
    libxext6 \
    libx11-6 \
    tmux \
    ros-melodic-rviz \
    eog \
 && rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

ADD /scripts /scripts

RUN bash /scripts/install_scripts_for_docker.bash
