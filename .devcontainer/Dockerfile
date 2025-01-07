# FROM mcr.microsoft.com/.devcontainers/base:jammy
# FROM ubuntu:22.04
FROM ubuntu:20.04@sha256:ed4a42283d9943135ed87d4ee34e542f7f5ad9ecf2f244870e23122f703f91c2

# ROS1 Noetic install
# =========================================

RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get install -y --no-install-recommends \
    python3 \ 
    python3-pip \
    # lsb-core for getting the ubuntu-version in scripts
    lsb-core \
    curl \
    gpg-agent \
    gnupg \
    build-essential

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && export DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full


# To be added to dev containear
# echo "source /opt/ros/noetic/setup.bash" >> /home/dev/.bashrc
# echo "source /opt/ros/noetic/setup.zsh" >> /home/dev/.zshrc


# Project Canopies / alitrack dependencies
# =========================================

RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get install -y --no-install-recommends \
    # Dependencies for alitrack_ packages git-repos in canopies 
    can-utils \
    python3-catkin-tools \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-socketcan-bridge \
    ros-noetic-twist-mux \
    ros-noetic-foxglove-bridge \
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-rosserial \
    ros-noetic-realsense2-camera \
    # Dependencies for packages in canopies-robot git-repo (pal-robotics)
    ros-noetic-realsense2-description \
    # Multi-master
    ros-noetic-fkie-multimaster \
    # Multi-master json paser in bash utility
    jq \
    # Dependencies for canopies ouister pacakge 
    # The package itself comes in a docker image and lars also made a wrapper - is that the way to go?
    build-essential \
    libeigen3-dev   \
    libjsoncpp-dev  \
    libspdlog-dev   \
    cmake \
    net-tools \
    iputils-ping



RUN python3 -m pip install --ignore-installed \
    # Pip dependencies for packages in alitrack_robot git-repo
    requests

RUN export DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get install -y --no-install-recommends \
    # gdb \
    # valgrind \
    # ssh \
    # flake8 \
    # pyyaml==5.3.1 \
    # bandit is a pyton security linter
    # bandit && \
    bash-completion \
    python3 \ 
    python3-pip \
    sudo \
    docker.io \
    git \
    # For checking shell scripts for warnings and bad coding
    shellcheck \
    # lsb-core for getting the ubuntu-version in scripts
    meld \
    # Tool to check urdf files (check_urdf)
    liburdfdom-tools \
    # Ping for dti_tools
    iputils-ping \
    # For scanning network for robots in dti_tool
    nmap \
    # For ip command (replaces net-tools with ifconfig)
    iproute2 \ 
    # The nano text editor
    nano 








# Set users
# Users from SSO system does not follow the convention of using same uid and gid
# Therefore this, for uid and gid mathing host user. Run 'id' in host terminal
RUN addgroup --quiet --gid 1000 dev_user_group 
RUN adduser --quiet --uid 1000 --ingroup dev_user_group --gecos "This user is awsome!" --shell /bin/bash --disabled-password dev
RUN usermod --append  --groups sudo dev

RUN echo 'dev:Docker!' | chpasswd
RUN echo 'root:Docker!' | chpasswd
WORKDIR /home/dev

# Intermidiate workaround to acces docker in docker
# Propper solution is to map user id and user group 
# to the same as host either using build_args, runtime or jinja docker files
RUN addgroup --quiet --gid 999 dockerdev 
RUN usermod --append  --groups dockerdev dev

#RUN python3 -m pip install --ignore-installed --break-system-packages -r  242-mobile-robotics/dti_tools/requirements.txt
#RUN source /opt/ros/noetic/setup.bash 

# Add sourcing ros to bashrc and of workspace
RUN echo "source /opt/ros/noetic/setup.bash " >> /home/dev/.bashrc
RUN echo "source /workspaces/docker-ros/catkin_ws/devel/setup.bash" >> /home/dev/.bashrc

# Cleanup
RUN rm -rf /root/.cache/pip/*. \
    && rm -rf /var/lib/apt/lists/*

# Set user and other stuff
USER dev

