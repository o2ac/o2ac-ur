
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
    dirmngr=2.2.19-3ubuntu2.1 \
    gnupg2=2.2.19-3ubuntu2.1 \
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
    ros-noetic-ros-core=1.5.0-1focal.20220107.010105 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS-desktop-full packages (including ROS, rqt, rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1focal.20220221.113435 \
    python3-rosdep=0.21.0-1 \
    python3-rosinstall=0.7.8-4 \
    python3-rosinstall-generator=0.1.22-1 \
    python3-wstool=0.1.18-2 \
    build-essential=12.8ubuntu1.1 \
	&& rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep

RUN rosdep init

################################################################################

# Install required packages/tools/dependencies for subsequent installations.
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https=2.0.6 \
    wget=1.20.3-1ubuntu2 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install common essentials,
RUN apt-get update && apt-cache policy git
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential=12.8ubuntu1.1 \
    cmake=3.16.3-1ubuntu1 \
    git=1:2.25.1-1ubuntu3.2 \
    python3-dev=3.8.2-0ubuntu2 \
    python3-numpy=1:1.17.4-5ubuntu3 \
    python3-pip=20.0.2-5ubuntu1.6 \
    python3-setuptools=45.2.0-1 \
    python3-scipy=1.3.3-3build1 \
	&& rm -rf /var/lib/apt/lists/*

# Pin pip version to make build more deterministic
RUN pip3 install --upgrade pip==22.0.4

################################################################################

# Install more common optional packages/tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    inetutils-ping=2:1.9.4-11ubuntu0.1 \
    iproute2=5.5.0-1ubuntu1 \
    gdb=9.2-0ubuntu1~20.04.1 \
    mesa-utils=8.4.0-1build1 \
    nano=4.8-1ubuntu1 \
    net-tools=1.60+git20180626.aebd88e-1ubuntu1 \
    openssh-client=1:8.2p1-4ubuntu0.4 \
    python3-catkin-tools=0.8.3-1 \
    python3-tk=3.8.10-0ubuntu1~20.04 \
    unzip=6.0-25ubuntu1 \
    vim=2:8.1.2269-1ubuntu5.7 \
    less=551-1ubuntu0.1 \
    usbutils=1:012-2 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# For copying UR scripts from the robots

RUN apt-get update && apt-get install -y --no-install-recommends \
    sshpass=1.06-1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install wstool to use .rosinstall file in the underlay workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-wstool=0.1.18-2 \
    clang-format-10=1:10.0.0-4ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install other dependencies of our workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-effort-controllers=0.19.0-1focal.20220221.104843 \
    ros-noetic-industrial-robot-status-controller=0.1.2-1focal.20220107.004524 \
    ros-noetic-socketcan-interface=0.8.5-1focal.20220106.233037 \
    ros-noetic-industrial-robot-status-interface=0.1.2-1focal.20220107.004233 \
    ros-noetic-soem=1.4.1003-1focal.20210423.224754 \
    ros-noetic-graph-msgs=0.1.0-2focal.20210423.223901 \
    ros-noetic-spacenav-node=1.15.1-1focal.20220107.001953 \
    ros-noetic-joy-teleop=0.4.0-1focal.20220107.002043 \
    ros-noetic-code-coverage=0.4.4-1focal.20210423.222045 \
    ros-noetic-panda-moveit-config=0.7.5-1focal.20220226.075123 \
    socat=1.7.3.3-2 \
    libqt5x11extras5-dev=5.12.8-0ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*	

################################################################################

# # Install JSK tools for visualization in Rviz
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-jsk-visualization \
# 	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for 'flexbe_app' ROS package.
# https://github.com/FlexBE/flexbe_app

# Gather dependencies from Ubuntu repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcanberra-gtk3-module=0.30-7ubuntu1 \
    libgconf2-dev=3.2.6-6ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install URe controller dependencies.

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-control=0.19.5-1focal.20220107.004158 \
    ros-noetic-robot-controllers=0.7.0-1focal.20220107.012709 \
    ros-noetic-velocity-controllers=0.19.0-1focal.20220221.105046 \
    ros-noetic-joint-trajectory-controller=0.19.0-1focal.20220221.105035 \
    ros-noetic-force-torque-sensor-controller=0.19.0-1focal.20220107.000731 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Add industrial_core submodule
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-industrial-core=0.7.2-1focal.20220221.113322 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for dynamixel and serial communication.
# # ros-noetic-qt-build \

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-dynamixel-sdk=3.7.51-4focal.20220106.235702 \
    ros-noetic-dynamixel-workbench=2.2.1-1focal.20220107.004242 \
    ros-noetic-dynamixel-workbench-msgs=2.0.2-2focal.20210629.160048 \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install pyserial==3.4

################################################################################

# Install pymodbus for the Robotiq gripper driver

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pymodbus=2.1.0+dfsg-2 \
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
    code=1.65.2-1646927742 \
	&& rm -rf /var/lib/apt/lists/*

# Set VS Code as the default text editor.
# https://code.visualstudio.com/docs/setup/linux#_setting-vs-code-as-the-default-text-editor
RUN update-alternatives --set editor /usr/bin/code

# Solve common error while deleting files from VS Code.
# https://code.visualstudio.com/docs/setup/linux#_debian-and-moving-files-to-trash
RUN apt-get update && apt-get install -y --no-install-recommends \
    gvfs-bin=1.44.1-1ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*

# Solve character rendering issues in GUI by adding missing Japanese fonts.
RUN apt-get update && apt-get install -y --no-install-recommends \
    fonts-takao=00303.01-3ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*

# Install a dependency that the next command requires, but VS Code seems to miss sometimes
RUN apt-get update && apt-get install -y --no-install-recommends \
    libxtst6=2:1.2.3-1 \
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
    exuberant-ctags=1:5.9~svn20110310-12 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install camera packages.

# librealsense2
RUN rm -f /etc/apt/sources.list.d/realsense-public.list
RUN echo 'deb http://librealsense.intel.com/Debian/apt-repo focal main' | tee /etc/apt/sources.list.d/realsense-public.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-dkms=1.3.18-0ubuntu1 \
    librealsense2-utils=2.50.0-0~realsense0.6128 \
    librealsense2-dev=2.50.0-0~realsense0.6128 \
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
RUN pip3 install cupy-cuda110==10.2.0 chainer==7.8.1 --ignore-installed

################################################################################

# Install gym for reinforcement learning
RUN pip3 install gym  --ignore-installed

################################################################################

# Install eigenpy 
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-eigenpy=2.6.11-1focal.20220226.071056 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Deep Learning
RUN pip3 install torch
RUN pip3 install --upgrade scipy

###############################################################################

# Install Python3 for open3d, and opencv
# Careful: This might set pip to use Python3 by default
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-dev=3.8.2-0ubuntu2 \
    python3-pip=20.0.2-5ubuntu1.6 \
    python3-setuptools=45.2.0-1 \
    python3-opencv=4.2.0+dfsg-5 \
	&& rm -rf /var/lib/apt/lists/*

# Required for opencv-python.  https://stackoverflow.com/questions/63448467/installing-opencv-fails-because-it-cannot-find-skbuild

RUN pip3 install open3d==0.12.0
RUN pip3 install opencv-python==4.5.1.48

################################################################################

# Install emacs
RUN apt-get update && apt-get install -y --no-install-recommends \
    emacs=1:26.3+1-1ubuntu2 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install cgal
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcgal-dev=5.0.2-3 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install matheval
RUN apt-get update && apt-get install -y --no-install-recommends \
    libmatheval-dev=1.1.11+dfsg-3build1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Dependency for ur_python_utilities
RUN pip3 install pyquaternion==0.9.9

# Utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    htop=2.2.0-2build1 \
    ros-noetic-rqt-multiplot=0.0.12-1focal.20220107.000756 \
    ros-noetic-trac-ik-python=1.6.6-1focal.20220107.012731 \
    ros-noetic-trac-ik-kinematics-plugin=1.6.6-1focal.20220218.100538 \
    ros-noetic-flexbe-behavior-engine=1.3.1-1focal.20220107.004342 \
    ros-noetic-gripper-action-controller=0.19.0-1focal.20220221.104848 \
    ros-noetic-jsk-rviz-plugins=2.1.8-1focal.20220221.111745 \
    ros-noetic-ros-numpy \
    python3-rosinstall=0.7.8-4 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Utilities to update the Dockerfile versions
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-apt=2.0.0ubuntu0.20.04.7 \
	&& rm -rf /var/lib/apt/lists/* \
    && pip3 install dockerfile_parse

RUN apt-get update && apt-get install -y --no-install-recommends \
    locate \
    && rm -rf /var/lib/apt/lists/* \
    && updatedb

################################################################################

# Speed up catkin build with ccache
RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache=3.7.7-1 \
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
