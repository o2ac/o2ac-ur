
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
	ros-melodic-desktop-full \
	&& rm -rf /var/lib/apt/lists/*

# Initialize 'rosdep'.
# http://wiki.ros.org/rosdep
RUN rosdep init

################################################################################

# Install MoveIt, because it seems to not be included in desktop-full?
RUN apt-get update && apt-get install ros-melodic-moveit -y

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
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Install JSK tools for visualization in Rviz
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-jsk-visualization=2.1.5-0bionic.20200229.074605

################################################################################

# Install dependencies for 'flexbe_app' ROS package.
# https://github.com/FlexBE/flexbe_app

# Gather dependencies from Ubuntu repositories.
RUN apt-get update && apt-get install -y --no-install-recommends \
  libcanberra-gtk3-module \
	libgconf2-dev \
	&& rm -rf /var/lib/apt/lists/*

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
	librealsense2-dbg

################################################################################

# Install dependencies for dynamixel and serial communication.
# # ros-melodic-qt-build \

RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-dynamixel-sdk \
	ros-melodic-dynamixel-workbench \
    ros-melodic-dynamixel-workbench-msgs
RUN pip install pyserial==3.4

################################################################################

# Install pymodbus for the Robotiq gripper driver

RUN apt-get update && apt-get install -y --no-install-recommends \
	python-pymodbus

################################################################################

# Install MoveIt! dependencies, because we build it from source
RUN apt-get update && apt-get install -y ros-melodic-moveit-resources

################################################################################

# Install python can module to use the Xela Robotics sensor.
RUN pip install python-can==3.1.1

################################################################################

# Install opencv for the cameras
RUN pip install opencv-python==4.0.0.21

################################################################################

# Install Tensorflow
# https://www.tensorflow.org/install
RUN pip install tensorflow-gpu==1.14.0

# Possible option: Install TensorRT
# RUN apt-get install -y --no-install-recommends \
#  nvinfer-runtime-trt-repo-ubuntu1804-5.0.2-ga-cuda10.0 \
#  libnvinfer-dev=5.0.2-1+cuda10.0

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

# Install Python3 & dependencies for PILCO
# This is required for running Python3 nodes in ROS

RUN apt-get update && apt-get install -y --no-install-recommends \
		python3-pip=9.0.1-2.3~ubuntu1.18.04.1 python3-yaml=3.12-1build2
RUN pip3 install gpflow==1.4.1
RUN pip3 install tensorflow-gpu==1.14.0
RUN pip3 install box2d-py==2.3.8  \
			rospkg==1.1.9  \
			catkin_pkg==0.4.13

################################################################################

# Install PyCharm
# https://www.jetbrains.com/pycharm/download/#section=linux
# FIXME: These instructions fail to find the graphics interface

# RUN wget -O - https://download.jetbrains.com/python/pycharm-community-2019.1.1.tar.gz \
# 	| tar xzf -
# RUN ./pycharm-community-2019.1.1/bin/pycharm.sh
# RUN rm pycharm-2019.1.1.tar.gz

################################################################################

# Install Anaconda
# https://docs.anaconda.com/anaconda/install/silent-mode/

# FIXME: These instructions might not be linked properly with all installed packages

# RUN wget -O /tmp/Anaconda_installer.sh https://repo.anaconda.com/archive/Anaconda3-2019.03-Linux-x86_64.sh
# RUN bash /tmp/Anaconda_installer.sh -b -p $HOME/anaconda
# ENV PATH="$HOME/anaconda/bin:$PATH"

################################################################################

RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-melodic-eigenpy

################################################################################

# Deep Learning
RUN pip install torch \
	tensorboardX
RUN pip install --upgrade scipy

################################################################################

# Add industrial_core submodule
RUN apt-get update && apt-get install -y ros-melodic-industrial-core=0.7.1-1bionic.20200218.163030

################################################################################

# Install emacs
RUN apt-get update && apt-get install -y --no-install-recommends \
	emacs \
	&& rm -rf /var/lib/apt/lists/*

################################################################################

# Define the working directory.
WORKDIR /root/

# Copy the helper scripts to the working directory.
COPY ./docker/scripts/. /root/scripts/
RUN chmod -R +x /root/scripts/*

# Setup the Bash shell environment with '/root/.bashrc'.
RUN cat /root/scripts/initialize-bash-shell.sh >> /root/.bashrc

# Overwrite the entry point of the parent image.
ENTRYPOINT []
RUN echo "\nsource /root/catkin_ws/devel/setup.bash\n" >> /root/.bashrc

# Enter the container with a Bash shell.
CMD ["/bin/bash"]
