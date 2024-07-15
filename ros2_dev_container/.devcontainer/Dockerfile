FROM ros:jazzy

RUN export DEBIAN_FRONTEND=noninteractive

# Add vscode user with same UID and GID as your host system, replace if one exists with same UID
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USER_NAME=ros2

COPY .env /tmp/.env
RUN . /tmp/.env && rm /tmp/.env && \
    if getent group $GROUP_ID > /dev/null; then \
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

# Install basic packages and set up locales
RUN apt update -y && apt-get -y install --no-install-recommends locales gettext \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_GB.UTF-8; update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
ENV LANG en_GB.UTF-8

# Install apt packages
COPY apt-packages /tmp/apt-packages
RUN apt-get update && \
    echo "packages: $(cut -d# -f1 </tmp/apt-packages | envsubst)" && \
    apt-get install -y $(cut -d# -f1 </tmp/apt-packages | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages

# Switch from root to user
USER $USER_NAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USER_NAME

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc


