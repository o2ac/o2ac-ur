
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
            libcudnn7=7.5.0.56-1+cuda10.1 \
            libcudnn7-dev=7.5.0.56-1+cuda10.1 && \
    rm -rf /var/lib/apt/lists/*

################################################################################

# Install ROS Melodic Morenia.
# http://wiki.ros.org/melodic/Installation/Ubuntu

# Update package list.
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list

# Add package keys.
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS-desktop-full packages (including ROS, rqt, rviz, and more).
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-desktop-full python-rosdep \
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
RUN apt-get update && apt-get install -y --no-install-recommends \
	build-essential \
	cmake \
	git \
	python-dev \
	python-numpy \
	python-pip \
	python-setuptools \
	python-scipy \
	&& rm -rf /var/lib/apt/lists/*

# Pin pip version to make build more deterministic
RUN pip install --upgrade pip==9.0.1

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
	python-catkin-tools \
	python-tk \
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
	ros-melodic-moveit-resources \
	ros-melodic-moveit \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install wstool to use .rosinstall file in the underlay workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
	python-wstool \
	python-catkin-tools \
	clang-format-10 \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install other dependencies of our workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-effort-controllers \
	ros-melodic-industrial-robot-status-controller \
	ros-melodic-socketcan-interface \
	ros-melodic-industrial-robot-status-interface \
	ros-melodic-soem \
	ros-melodic-graph-msgs \
	ros-melodic-spacenav-node \
	ros-melodic-joy-teleop \
	ros-melodic-code-coverage \
	ros-melodic-panda-moveit-config \
	socat \
	libqt5x11extras5-dev \
	&& rm -rf /var/lib/apt/lists/*	

################################################################################

# Install JSK tools for visualization in Rviz
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-jsk-visualization \
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
	ros-melodic-ros-control \
	ros-melodic-robot-controllers \
	ros-melodic-velocity-controllers \
	ros-melodic-joint-trajectory-controller \
   	ros-melodic-force-torque-sensor-controller \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Add industrial_core submodule
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-industrial-core \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install dependencies for dynamixel and serial communication.
# # ros-melodic-qt-build \

RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-dynamixel-sdk \
	ros-melodic-dynamixel-workbench \
    ros-melodic-dynamixel-workbench-msgs \
	&& rm -rf /var/lib/apt/lists/*

RUN pip install pyserial==3.4

################################################################################

# Install pymodbus for the Robotiq gripper driver

RUN apt-get update && apt-get install -y --no-install-recommends \
	python-pymodbus \
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
RUN apt-get update && apt-get install libxtst6

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
	exuberant-ctags \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install the SSH deploy key of the repository hosted on GitLab.com.

# Copy the SSH deploy key from the sources on the host to the container.
COPY ./.ssh/. /root/.ssh/
RUN chmod 700 /root/.ssh/ \
	&& chmod 600 /root/.ssh/o2ac-ur-id_rsa

# Add GitLab.com as a known host using the SSH deploy key.
RUN echo "Host gitlab.com" > /root/.ssh/config \
	&& echo "  Hostname gitlab.com" >> /root/.ssh/config \
	&& echo "  IdentityFile /root/.ssh/o2ac-ur-id_rsa" >> /root/.ssh/config \
	&& echo "  StrictHostKeyChecking no" >> /root/.ssh/config \
	&& echo "  User git" >> /root/.ssh/config

################################################################################

# Install camera packages.

# librealsense2
RUN rm -f /etc/apt/sources.list.d/realsense-public.list
RUN echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main' | tee /etc/apt/sources.list.d/realsense-public.list
# RUN apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
RUN apt-get update && apt-get install -y --no-install-recommends \
	librealsense2-dkms \
	librealsense2-utils \
	librealsense2-dev \
	librealsense2-udev-rules \
	librealsense2-dbg \
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
	ros-melodic-eigenpy \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Deep Learning
RUN pip install torch
RUN pip install --upgrade scipy

################################################################################

# Install emacs
RUN apt-get update && apt-get install -y --no-install-recommends \
	emacs \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Setup the Bash shell environment (part 1)

# Define the working directory.
WORKDIR /root/

################################################################################

# Copy the helper script
COPY ./docker/scripts/install-phoxi-control.sh /root/scripts/install-phoxi-control.sh
RUN chmod -R +x /root/scripts/install-phoxi-control.sh

# Install PhoXiControl (for the AIST setup)
RUN /root/scripts/install-phoxi-control.sh

# Copy the helper script
COPY ./docker/scripts/install-photoneo-localization.sh /root/scripts/install-photoneo-localization.sh
RUN chmod -R +x /root/scripts/install-photoneo-localization.sh

# Install PhotoneoLocalizationSDK
RUN apt-get update && apt-get install -y --no-install-recommends \
	libssh2-1 \
	expect
RUN /root/scripts/install-photoneo-localization.sh

################################################################################

# Setup the Bash shell environment (part 2)

# Copy the helper scripts to the working directory.
COPY ./docker/scripts/. /root/scripts/
RUN chmod -R +x /root/scripts/*

# Setup the Bash shell environment with '/root/.bashrc'.
RUN echo "source /root/scripts/initialize-bash-shell.sh" >> /root/.bashrc
RUN echo "\nsource /root/catkin_ws/devel/setup.bash\n" >> /root/.bashrc

# Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

# Traceability.
ARG DOCKERFILE_COMMIT_SHORT_SHA
ENV DOCKER_IMAGE_VERSION_O2AC_UR=${DOCKERFILE_COMMIT_SHORT_SHA}

# Overwrite the entry point of the parent image.
ENTRYPOINT []

# Enter the container with a Bash shell.
CMD ["/bin/bash"]
