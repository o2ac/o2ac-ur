
# General guidelines and recommendations for writing Dockerfile:
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/

################################################################################

# Install Ubuntu Bionic Beaver (18.04) with CUDA and OpenGL support.
# https://hub.docker.com/r/nvidia/cudagl/
# https://gitlab.com/nvidia/cudagl
# https://github.com/NVIDIA/libglvnd

# Pull official parent image from Nvidia repository.
FROM nvidia/cudagl:11.0-devel-ubuntu20.04
# 10.1-devel-ubuntu20.04

# Avoid user interaction during install of tzdata
# https://askubuntu.com/questions/909277/avoiding-user-interaction-with-tzdata-when-installing-certbot-in-a-docker-contai
ENV TZ=Asia
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install Nvidia cuDNN library missing from parent image.
# https://gitlab.com/nvidia/cuda/blob/ubuntu18.04/10.1/devel/cudnn7/Dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=8.3.2.44-1+cuda11.5 \
    libcudnn8-dev=8.3.2.44-1+cuda11.5 \
    && rm -rf /var/lib/apt/lists/*

################################################################################

# Install ROS Noetic Ninjemys.
# http://wiki.ros.org/noetic/Installation/Ubuntu

# setup timezone
# RUN echo 'Etc/UTC' > /etc/timezone && \
#     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#     apt-get update && \
#     apt-get install -q -y --no-install-recommends tzdata && \
#     rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV ROS_DISTRO=noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install ROS-desktop-full packages (including ROS, rqt, rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
	&& rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep
RUN rosdep init

################################################################################

# Install required packages/tools/dependencies for subsequent installations.
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https \
    wget \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install common essentials,
RUN apt-get update && apt-cache policy git
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-setuptools \
    python3-scipy \
	&& rm -rf /var/lib/apt/lists/*

# Pin pip version to make build more deterministic
RUN pip3 install --upgrade pip==22.0.4

################################################################################

# Install more common optional packages/tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    inetutils-ping \
    iproute2 \
    gdb \
    mesa-utils \
    nano \
    net-tools \
    openssh-client \
    python3-catkin-tools \
    python3-tk \
    unzip \
    vim \
    less \
    usbutils \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# For copying UR scripts from the robots

RUN apt-get update && apt-get install -y --no-install-recommends \
	sshpass \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install MoveIt dependencies because we build it from source.
# Also install the binaries to pull in the dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \ 
    ros-noetic-moveit-resources \
    ros-noetic-moveit \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install wstool to use .rosinstall file in the underlay workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-wstool \
    clang-format-10 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install other dependencies of our workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-effort-controllers \
    ros-noetic-industrial-robot-status-controller \
    ros-noetic-socketcan-interface \
    ros-noetic-industrial-robot-status-interface \
    ros-noetic-soem \
    ros-noetic-graph-msgs \
    ros-noetic-spacenav-node \
    ros-noetic-joy-teleop \
    ros-noetic-code-coverage \
    ros-noetic-panda-moveit-config \
    socat \
    libqt5x11extras5-dev \
	&& rm -rf /var/lib/apt/lists/*	

################################################################################

# Install JSK tools for visualization in Rviz
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-jsk-visualization \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for 'flexbe_app' ROS package.
# https://github.com/FlexBE/flexbe_app

# Gather dependencies from Ubuntu repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcanberra-gtk3-module \
    libgconf2-dev \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install URe controller dependencies.

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-control \
    ros-noetic-robot-controllers \
    ros-noetic-velocity-controllers \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-force-torque-sensor-controller \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Add industrial_core submodule
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-industrial-core \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for dynamixel and serial communication.
# # ros-noetic-qt-build \

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-dynamixel-sdk \
    ros-noetic-dynamixel-workbench \
    ros-noetic-dynamixel-workbench-msgs \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install pyserial==3.4

################################################################################

# Install pymodbus for the Robotiq gripper driver

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pymodbus \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install python can module to use the Xela Robotics sensor.
RUN pip3 install python-can==3.1.1

################################################################################

# Install Microsoft VS Code.
# https://code.visualstudio.com/docs/setup/linux#_installation

# Install the Microsoft repository key.
RUN curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg \
	&& mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg \
	&& echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list

# Update cache and install VS Code as a package.
RUN apt-get update && apt-get install -y --no-install-recommends \
    code \
	&& rm -rf /var/lib/apt/lists/*

# Set VS Code as the default text editor.
# https://code.visualstudio.com/docs/setup/linux#_setting-vs-code-as-the-default-text-editor
RUN update-alternatives --set editor /usr/bin/code

# Solve common error while deleting files from VS Code.
# https://code.visualstudio.com/docs/setup/linux#_debian-and-moving-files-to-trash
RUN apt-get update && apt-get install -y --no-install-recommends \
    gvfs-bin \
	&& rm -rf /var/lib/apt/lists/*

# Solve character rendering issues in GUI by adding missing Japanese fonts.
RUN apt-get update && apt-get install -y --no-install-recommends \
    fonts-takao \
	&& rm -rf /var/lib/apt/lists/*

# Install a dependency that the next command requires, but VS Code seems to miss sometimes
RUN apt-get update && apt-get install -y --no-install-recommends \
    libxtst6 \
	&& rm -rf /var/lib/apt/lists/*

# Install additional recommended VS Code extensions.
# https://code.visualstudio.com/docs/editor/extension-gallery#_command-line-extension-management
RUN code --install-extension ms-python.python --user-data-dir=/root/.vscode/ \
	&& code --install-extension ms-vscode.cpptools --user-data-dir=/root/.vscode/ \
	&& code --install-extension ms-azuretools.vscode-docker --user-data-dir=/root/.vscode/ \
	&& code --install-extension streetsidesoftware.code-spell-checker --user-data-dir=/root/.vscode/ \
	&& code --install-extension eamodio.gitlens --user-data-dir=/root/.vscode/ \
	&& code --install-extension mhutchie.git-graph --user-data-dir=/root/.vscode/ \
	&& code --install-extension ms-iot.vscode-ros --user-data-dir=/root/.vscode/ \
	&& code --install-extension ajshort.msg --user-data-dir=/root/.vscode/

# Install Python linter as recommended by Python extension.
RUN /usr/bin/python3 -m pip install -U \
	pylint \
	autopep8

# Install Exuberant Ctags as recommended by Python extension.
RUN apt-get update && apt-get install -y --no-install-recommends \
    exuberant-ctags \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install camera packages.

# librealsense2
RUN rm -f /etc/apt/sources.list.d/realsense-public.list
RUN echo 'deb http://librealsense.intel.com/Debian/apt-repo focal main' | tee /etc/apt/sources.list.d/realsense-public.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install opencv for the cameras
RUN pip3 install opencv-python==4.1.2.30

################################################################################

# Install keras
# https://keras.io/#installation
RUN pip3 install keras==2.2.4

################################################################################

# Install CuPy for chainer hardware acceleration
# https://docs-cupy.chainer.org/en/latest/install.html
# And chainer
# https://docs.chainer.org/en/stable/install.html
RUN pip3 install cupy-cuda110==10.2.0 chainer==7.8.1

################################################################################

# Install gym for reinforcement learning
RUN pip3 install gym

################################################################################

# Install eigenpy 
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-eigenpy \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Deep Learning
RUN pip3 install torch
RUN pip3 install --upgrade scipy

################################################################################

# Install Python3 for open3d, and opencv
# Careful: This might set pip to use Python3 by default
RUN apt-get update && apt-get install -y --no-install-recommends \
	python3-dev \
    python3-pip \
    python3-setuptools \
    python3-opencv \
	&& rm -rf /var/lib/apt/lists/*

# Required for opencv-python.  https://stackoverflow.com/questions/63448467/installing-opencv-fails-because-it-cannot-find-skbuild

RUN pip3 install open3d==0.12.0
RUN pip3 install opencv-python==4.5.1.48
RUN pip3 install rospkg==1.2.10

################################################################################

# Install emacs
RUN apt-get update && apt-get install -y --no-install-recommends \
    emacs \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install cgal
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcgal-dev \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install matheval
RUN apt-get update && apt-get install -y --no-install-recommends \
	libmatheval-dev \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Dependency for ur_python_utilities
RUN pip3 install pyquaternion==0.9.9

# Utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    htop \
    ros-noetic-rqt-multiplot \
    ros-noetic-trac-ik-python \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-flexbe-behavior-engine \
    ros-noetic-gripper-action-controller \
    ros-noetic-jsk-rviz-plugins \
    python3-rosinstall \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Utilities to update the Dockerfile versions
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-apt \
	&& rm -rf /var/lib/apt/lists/* \
    && pip3 install dockerfile_parse

################################################################################

# Speed up catkin build with ccache
RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache \
    && rm -rf /var/lib/apt/lists/*
RUN echo 'export PATH=/usr/lib/ccache:$PATH' >> $HOME/.bashrc

################################################################################

# Setup the Bash shell environment

# Define the working directory.
WORKDIR /root/o2ac-ur/

# Copy the helper scripts to the working directory.
COPY ./docker/o2ac-dev/scripts/. /root/o2ac-ur/docker/o2ac-dev/scripts/
RUN chmod -R +x /root/o2ac-ur/docker/o2ac-dev/scripts/*

# Setup the Bash shell environment with '/root/.bashrc'.
RUN echo "source /root/o2ac-ur/docker/o2ac-dev/scripts/initialize-bash-shell.sh" >> /root/.bashrc
RUN echo "\nsource /root/o2ac-ur/catkin_ws/devel/setup.bash\n" >> /root/.bashrc

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

# Traceability.
ARG DOCKERFILE_COMMIT_SHORT_SHA
ENV DOCKER_IMAGE_VERSION_O2AC_UR=${DOCKERFILE_COMMIT_SHORT_SHA}

# Overwrite the entry point of the parent image.
ENTRYPOINT []

# Enter the container with a Bash shell.
CMD ["/bin/bash"]
