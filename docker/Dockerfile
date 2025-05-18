FROM ros:humble

# we want the familiar bash for everything, not sh.
# this makes source-ing work
SHELL ["/bin/bash", "-c"]

# this allows the containers to connect to the host ros2
# use --network=host --ipc=host --pid=host with run 
# and export ROS_DOMAIN_ID=42 on the host
ENV ROS_DOMAIN_ID 42
ENV ROS_LOCALHOST_ONLY 0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# update things
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y python3-pip \
    apt-utils \
    vim tmux \
    sudo git \
    ros-humble-mqtt-client \
    net-tools iproute2 iputils-ping \
    && pip install --upgrade pip==25.0 \
    && pip install --no-input setuptools==58.2.0 filterpy pymavlink mavproxy pyserial \
    && rm -rf /var/lib/apt/lists/*

# install rosdep dependencies and then delete the source
# you should volume mount your source code into the container during runtime instead!
COPY src /tmp/src
RUN apt-get update && rosdep update \
    && cd /tmp/src/tuper && rosdep install --from-paths \
    arduagent \
    BehaviorTree.CPP BehaviorTree.ROS2 \
    formation_controller \
    path_planner \
    position_filter \
    tuper_btcpp \
    tuper_interfaces --ignore-src -y \
    && rm -rf /tmp/src

# create a non-root user for most things, including ros2
ARG UID=1000
ARG GID=1000
ARG USERNAME=smarc2user
RUN adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} --uid ${GID:=1000} ${USERNAME} \
&& usermod -aG sudo ${USERNAME}

# Allow the user to use sudo without a password
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

WORKDIR /home/${USERNAME}

# make everything we installed into the user's home directory 
# actually owned by the user...
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

RUN mkdir colcon_ws
WORKDIR /home/${USERNAME}/colcon_ws

# Edit ~/.bashrc contents
COPY src/tuper/docker/custom_bashrc.sh /tmp/custom_bashrc.sh
RUN cat /tmp/custom_bashrc.sh >> /home/${USERNAME}/.bashrc \
    && source /home/${USERNAME}/.bashrc \
    && rm /tmp/custom_bashrc.sh

# finally switch to the user we created for normal use
USER ${USERNAME}

ENTRYPOINT ["/bin/bash"]



