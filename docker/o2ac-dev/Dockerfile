
# General guidelines and recommendations for writing Dockerfile:
# https://docs.docker.com/engine/userguide/eng-image/dockerfile_best-practices/

################################################################################

# Install Ubuntu Bionic Beaver (18.04) with CUDA and OpenGL support.
# https://hub.docker.com/r/nvidia/cudagl/
# https://gitlab.com/nvidia/cudagl
# https://github.com/NVIDIA/libglvnd

# Pull official parent image from Nvidia repository.
FROM nvidia/cudagl:10.1-devel-ubuntu18.04

# Avoid user interaction during install of tzdata
# https://askubuntu.com/questions/909277/avoiding-user-interaction-with-tzdata-when-installing-certbot-in-a-docker-contai
ENV TZ=Asia
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install Nvidia cuDNN library missing from parent image.
# https://gitlab.com/nvidia/cuda/blob/ubuntu18.04/10.1/devel/cudnn7/Dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn7=7.6.5.32-1+cuda10.2 \
    libcudnn7-dev=7.6.5.32-1+cuda10.2 \
    && rm -rf /var/lib/apt/lists/*

################################################################################

# Install ROS Melodic Morenia.
# http://wiki.ros.org/melodic/Installation/Ubuntu

# Update package list.
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Add package keys.
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS-desktop-full packages (including ROS, rqt, rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full=1.4.1-0bionic.20210921.232208 \
    python-rosdep=0.21.0-1 \
	&& rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep
RUN rosdep init

################################################################################

# Install required packages/tools/dependencies for subsequent installations.
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https=1.6.14 \
    wget=1.19.4-1ubuntu2.2 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install common essentials,
RUN apt-get update && apt-cache policy git
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential=12.4ubuntu1 \
    cmake=3.10.2-1ubuntu2.18.04.2 \
    git=1:2.17.1-1ubuntu0.9 \
    python-dev=2.7.15~rc1-1 \
    python-numpy=1:1.13.3-2ubuntu1 \
    python-pip=9.0.1-2.3~ubuntu1.18.04.5 \
    python-setuptools=39.0.1-2 \
    python-scipy=0.19.1-2ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*

# Pin pip version to make build more deterministic
RUN pip install --upgrade pip==9.0.1

################################################################################

# Install more common optional packages/tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    inetutils-ping=2:1.9.4-3ubuntu0.1 \
    iproute2=4.15.0-2ubuntu1.3 \
    gdb=8.1.1-0ubuntu1 \
    mesa-utils=8.4.0-1 \
    nano=2.9.3-2 \
    net-tools=1.60+git20161116.90da8a0-1ubuntu1 \
    openssh-client=1:7.6p1-4ubuntu0.5 \
    python-catkin-tools=0.6.1-1 \
    python-tk=2.7.17-1~18.04 \
    unzip=6.0-21ubuntu1.1 \
    vim=2:8.0.1453-1ubuntu1.6 \
    less=487-0.1 \
    usbutils=1:007-4build1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# For copying UR scripts from the robots

RUN apt-get update && apt-get install -y --no-install-recommends \
	sshpass=1.06-1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install MoveIt dependencies because we build it from source.
# Also install the binaries to pull in the dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \ 
    ros-melodic-moveit-resources=0.7.3-1bionic.20210921.221050 \
    ros-melodic-moveit=1.0.8-1bionic.20210922.000107 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install wstool to use .rosinstall file in the underlay workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-wstool=0.1.17-1 \
    clang-format-10=1:10.0.0-4ubuntu1~18.04.2 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install other dependencies of our workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-effort-controllers=0.17.2-1bionic.20210921.221828 \
    ros-melodic-industrial-robot-status-controller=0.1.2-1bionic.20210921.221524 \
    ros-melodic-socketcan-interface=0.8.5-1bionic.20210921.180404 \
    ros-melodic-industrial-robot-status-interface=0.1.2-1bionic.20210921.203234 \
    ros-melodic-soem=1.4.1003-1bionic.20201015.050528 \
    ros-melodic-graph-msgs=0.1.0-1bionic.20210505.012422 \
    ros-melodic-spacenav-node=1.14.0-1bionic.20210921.210836 \
    ros-melodic-joy-teleop=0.3.1-1bionic.20210921.205844 \
    ros-melodic-code-coverage=0.4.3-1bionic.20200803.132715 \
    ros-melodic-panda-moveit-config=0.7.4-1bionic.20210921.234706 \
    socat=1.7.3.2-2ubuntu2 \
    libqt5x11extras5-dev=5.9.5-0ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*	

################################################################################

# Install JSK tools for visualization in Rviz
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-jsk-visualization=2.1.7-2bionic.20210922.000226 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for 'flexbe_app' ROS package.
# https://github.com/FlexBE/flexbe_app

# Gather dependencies from Ubuntu repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcanberra-gtk3-module=0.30-5ubuntu1 \
    libgconf2-dev=3.2.6-4ubuntu1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install URe controller dependencies.

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-ros-control=0.18.4-1bionic.20210921.220930 \
    ros-melodic-robot-controllers=0.6.0-0bionic.20210921.222303 \
    ros-melodic-velocity-controllers=0.17.2-1bionic.20210921.221837 \
    ros-melodic-joint-trajectory-controller=0.17.2-1bionic.20210921.220639 \
    ros-melodic-force-torque-sensor-controller=0.17.2-1bionic.20210921.221513 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Add industrial_core submodule
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-industrial-core=0.7.3-1bionic.20210921.231521 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for dynamixel and serial communication.
# # ros-melodic-qt-build \

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-dynamixel-sdk=3.7.51-4bionic.20210921.181333 \
    ros-melodic-dynamixel-workbench=2.2.0-0bionic.20210921.212132 \
    ros-melodic-dynamixel-workbench-msgs=2.0.1-0bionic.20210505.011002 \
	&& rm -rf /var/lib/apt/lists/*

RUN pip install pyserial==3.4

################################################################################

# Install pymodbus for the Robotiq gripper driver

RUN apt-get update && apt-get install -y --no-install-recommends \
    python-pymodbus=1.3.2-1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install python can module to use the Xela Robotics sensor.
RUN pip install python-can==3.1.1

################################################################################

# Install Microsoft VS Code.
# https://code.visualstudio.com/docs/setup/linux#_installation

# Install the Microsoft repository key.
RUN curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg \
	&& mv microsoft.gpg /etc/apt/trusted.gpg.d/microsoft.gpg \
	&& echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list

# Update cache and install VS Code as a package.
RUN apt-get update && apt-get install -y --no-install-recommends \
    code=1.61.1-1634175470 \
	&& rm -rf /var/lib/apt/lists/*

# Set VS Code as the default text editor.
# https://code.visualstudio.com/docs/setup/linux#_setting-vs-code-as-the-default-text-editor
RUN update-alternatives --set editor /usr/bin/code

# Solve common error while deleting files from VS Code.
# https://code.visualstudio.com/docs/setup/linux#_debian-and-moving-files-to-trash
RUN apt-get update && apt-get install -y --no-install-recommends \
    gvfs-bin=1.36.1-0ubuntu1.3.3 \
	&& rm -rf /var/lib/apt/lists/*

# Solve character rendering issues in GUI by adding missing Japanese fonts.
RUN apt-get update && apt-get install -y --no-install-recommends \
    fonts-takao=00303.01-2ubuntu1 \
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
RUN /usr/bin/python -m pip install -U \
	pylint \
	autopep8

# Install Exuberant Ctags as recommended by Python extension.
RUN apt-get update && apt-get install -y --no-install-recommends \
    exuberant-ctags=1:5.9~svn20110310-11 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install camera packages.

# librealsense2
RUN rm -f /etc/apt/sources.list.d/realsense-public.list
RUN echo 'deb http://librealsense.intel.com/Debian/apt-repo bionic main' | tee /etc/apt/sources.list.d/realsense-public.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN apt-get update && apt-get install -y --no-install-recommends \
    librealsense2-dkms=1.3.18-0ubuntu1 \
    librealsense2-utils=2.49.0-0~realsense0.5306 \
    librealsense2-dev=2.49.0-0~realsense0.5306 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install opencv for the cameras
RUN pip install opencv-python==4.0.0.21

################################################################################

# Install keras
# https://keras.io/#installation
RUN pip install keras==2.2.4

################################################################################

# Install CuPy for chainer hardware acceleration
# https://docs-cupy.chainer.org/en/latest/install.html
# And chainer
# https://docs.chainer.org/en/stable/install.html
RUN pip install cupy-cuda101==5.4.0 chainer==5.4.0

################################################################################

# Install gym for reinforcement learning
RUN pip install gym

################################################################################

# Install eigenpy 
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-eigenpy=2.6.8-1bionic.20210907.141314 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Deep Learning
RUN pip install torch
RUN pip install --upgrade scipy

################################################################################

# Install Python3 for open3d, and opencv
# Careful: This might set pip to use Python3 by default
RUN apt-get update && apt-get install -y --no-install-recommends \
	python3.6-dev=3.6.9-1~18.04ubuntu1.4 \
    python3-pip=9.0.1-2.3~ubuntu1.18.04.5 \
    python3-setuptools=39.0.1-2 \
    python3-opencv=3.2.0+dfsg-4ubuntu0.1 \
	&& rm -rf /var/lib/apt/lists/*

# Required for opencv-python.  https://stackoverflow.com/questions/63448467/installing-opencv-fails-because-it-cannot-find-skbuild

RUN pip3 install --upgrade pip==21.0.1

RUN pip3 install open3d==0.12.0
RUN pip3 install opencv-python==4.5.1.48
RUN pip3 install rospkg==1.2.10

################################################################################

# Install emacs
RUN apt-get update && apt-get install -y --no-install-recommends \
    emacs=47.0 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install cgal
RUN apt-get update && apt-get install -y --no-install-recommends \
    libcgal-dev=4.11-2build1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install matheval
RUN apt-get update && apt-get install -y --no-install-recommends \
	libmatheval-dev=1.1.11+dfsg-3build1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Dependency for ur_python_utilities
RUN pip2 install pyquaternion==0.9.9

# Utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    htop=2.1.0-3 \
    ros-melodic-rqt-multiplot=0.0.10-0bionic.20210921.205109 \
    ros-melodic-trac-ik-python=1.5.1-1bionic.20210921.222641 \
    ros-melodic-trac-ik-kinematics-plugin=1.5.1-1bionic.20210921.224102 \
    ros-melodic-flexbe-behavior-engine=1.3.1-1bionic.20210921.221501 \
    ros-melodic-gripper-action-controller=0.17.2-1bionic.20210921.220651 \
    ros-melodic-jsk-rviz-plugins=2.1.7-2bionic.20210921.230839 \
    python-rosinstall=0.7.8-1 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Utilities to update the Dockerfile versions
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-apt=1.6.5ubuntu0.7 \
	&& rm -rf /var/lib/apt/lists/* \
    && pip3 install dockerfile_parse

################################################################################

# Speed up catkin build with ccache
RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache=3.4.1-1 \
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
