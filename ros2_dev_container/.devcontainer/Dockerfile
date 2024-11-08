ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO} AS ros2_base
ENV ROS_DISTRO=${ROS_DISTRO}

RUN export DEBIAN_FRONTEND=noninteractive

# Install basic packages and set up locales
RUN apt update -y && apt-get -y install --no-install-recommends locales gettext \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_GB.UTF-8; update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
ENV LANG en_GB.UTF-8

# Install apt packages
COPY apt-base-packages /tmp/apt-base-packages
RUN apt-get update && \
    apt-get install -y $(cut -d# -f1 </tmp/apt-base-packages | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-base-packages

## Computer vision stage
FROM ros2_base AS ros2_cv

# Install apt packages
COPY apt-cv-packages /tmp/apt-cv-packages
RUN apt-get update && \
    apt-get install -y $(cut -d# -f1 </tmp/apt-cv-packages | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-cv-packages

## Development stage
FROM ros2_cv AS ros2_dev

# Install apt packages
COPY apt-dev-packages /tmp/apt-dev-packages
RUN apt-get update && \
    apt-get install -y $(cut -d# -f1 </tmp/apt-dev-packages | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-dev-packages

# Add user with same UID and GID as your host system, replace if one exists with same UID
ARG USER_NAME=ros2
ARG USER_ID
ARG GROUP_NAME
ARG GROUP_ID

RUN if getent group $GROUP_ID > /dev/null; then \
    OLD_GROUP_NAME=$(getent group $GROUP_ID |  cut -d: -f1); \
    groupmod --new-name $USER_NAME $OLD_GROUP_NAME; \
    else \
    groupadd --gid $GROUP_ID $USER_NAME; \
    fi \
    && if getent passwd $USER_ID > /dev/null; then \
    OLD_USER_NAME=$(getent passwd $USER_ID |  cut -d: -f1); \
    usermod -l $USER_NAME $OLD_USER_NAME; \
    usermod -d /home/$USER_NAME -m $USER_NAME; \
    else \
    useradd -s /bin/bash --uid $USER_ID --gid $GROUP_ID -m $USER_NAME; \
    fi \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USER_NAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USER_NAME \
    && chmod 0440 /etc/sudoers.d/$USER_NAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USER_NAME

RUN rosdep update

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER_NAME}/.bashrc