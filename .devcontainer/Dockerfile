#FROM nvidia/cuda:12.8.1-devel-ubuntu22.04
FROM ubuntu:22.04

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID


# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi


# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
ENV SHELL=/bin/bash


# Ref: https://roboticseabass.com/2021/04/21/docker-and-ros/
 
# Minimal setup
RUN apt-get update --fix-missing \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales
 
# Following the ROS2 Humble official setup guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Locale Setup
RUN locale  # check for UTF-8

RUN apt update && apt install -y --no-install-recommends locales 
RUN locale-gen en_US en_US.UTF-8 
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
RUN export LANG=en_US.UTF-8 
RUN locale  # verify settings


# Update and install necessary packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    sudo \
    locales \
    lsb-release \
    software-properties-common \
    curl \
    python3-pip \
    python3-venv \
    git \
    gdb \
    gdbserver \
    gedit \
    xtermcontrol \
    nano \
    xclip

# Install ssh if not already installed
RUN if [ ! -x "$(command -v ssh)" ]; then \
      apt-get update -qq \
      && apt-get install -y --no-install-recommends \
        openssh-client \
      && rm -rf /var/lib/apt/lists/*; \
    fi

RUN echo "StrictHostKeyChecking no" >> /etc/ssh/ssh_config

# Set shell to bash
ENV SHELL=/bin/bash

# Add repository for ROS2 packages
RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Install ROS2 Humble
RUN apt update && apt install -y --no-install-recommends ros-humble-desktop
RUN apt update && apt install -y --no-install-recommends ros-dev-tools
RUN apt update &&  apt install -y --no-install-recommends python3-rosdep

# Initialize rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

# Copy and run root setup scripts
COPY setup_scripts/root /scripts_root
RUN find /scripts_root -type f -name '*.sh' -exec sed -i 's/\r$//' {} +
RUN /bin/bash /scripts_root/_source_all.sh


# Setup
RUN echo "\n\
source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

RUN echo "\n\
if [ -f /home/ws/install/setup.bash ]; then source /home/ws/install/setup.bash; fi\n\
export ROS_WORKSPACE=/home/ws\n\
export ROS_LOCATIONS='root=/opt/ros/humble/share:dev=/home/ws/install:src=/home/ws/src'\n\
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib\n\
export MUJOCO_GL=egl" >> /home/$USERNAME/.bashrc

RUN echo "\n\
# Enable colcon autocompletion\n\
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\n\
alias cbp='colcon build --packages-select'" >> /home/$USERNAME/.bashrc

# Set ROS2 Environment Variables
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble

# Add user to necessary groups for device access
# check first if groups exist
ARG RENDER_GROUP_ID=109
ARG VIDEO_GROUP_ID=44
ARG INPUT_GROUP_ID=107

RUN getent group render || groupadd --gid $RENDER_GROUP_ID render
RUN getent group video || groupadd --gid $VIDEO_GROUP_ID video
RUN getent group input || groupadd --gid $INPUT_GROUP_ID input
RUN usermod -aG render,video,input $USERNAME



# Switch to non-root user
USER $USERNAME

# Copy and run user setup scripts
COPY --chown=$USERNAME:$USERNAME setup_scripts/user /scripts_user
RUN find /scripts_user -type f -name '*.sh' -exec sed -i 's/\r$//' {} +
RUN /bin/bash /scripts_user/_source_all.sh

# Set working directory
WORKDIR /home/ws

# Start with bash
CMD ["/bin/bash"]
