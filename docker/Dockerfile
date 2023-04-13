FROM nvidia/cuda:11.5.1-devel-ubuntu20.04
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
        wget \
        curl \
        git \
        vim \
        python3-pip \
        lsb-release \
        gnupg \
        avahi-autoipd \
        avahi-daemon \
        avahi-utils

# ROS setting
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros-focal.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update && \
    apt install -y \
        python3-catkin-pkg-modules \
        python3-rosdistro-modules \
        python3-rospkg-modules
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# # RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update && \
    apt install -y \
        ros-noetic-desktop-full

RUN apt install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        build-essential \
        python3-catkin-tools

RUN rosdep init && rosdep update

# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#         python-rosinstall \
#         python-catkin-tools

RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    bash -c "source /opt/ros/noetic/setup.bash; catkin_make"


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git-core \
        python3-wstool \
        python3-vcstools \
        ros-noetic-control-msgs \
        ros-noetic-xacro \
        ros-noetic-tf2-ros \
        ros-noetic-rviz \
        ros-noetic-cv-bridge \
        ros-noetic-actionlib \
        ros-noetic-actionlib-msgs \
        ros-noetic-dynamic-reconfigure \
        ros-noetic-trajectory-msgs \
        ros-noetic-rospy-message-converter


RUN apt-get update && \
    apt-get install -y \
        ros-noetic-ros-control \
        ros-noetic-control-toolbox \
        ros-noetic-realtime-tools \
        ros-noetic-ros-controllers \
        ros-noetic-xacro \
        ros-noetic-tf-conversions \
        ros-noetic-kdl-parser \
        ros-noetic-moveit-resources-prbt-moveit-config \
        ros-noetic-pilz-industrial-motion-planner 


RUN source /root/.bashrc && \
    pip install matplotlib && \
    pip install torch==1.11.0+cu115 torchvision==0.12.0+cu115 torchaudio==0.11.0+cu115 -f https://download.pytorch.org/whl/cu115/torch_stable.html

# Copy the helper scripts to the working directory.
COPY ./docker/scripts/. /root/scripts/
RUN chmod -R +x /root/scripts/*

# # Setup the Bash shell environment with '/root/.bashrc'.
RUN echo "source /root/scripts/initialize-bash-shell.sh" >> /root/.bashrc

# # Force color prompt in terminal.
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /root/.bashrc

RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root

ENTRYPOINT []

CMD ["/bin/bash"]

