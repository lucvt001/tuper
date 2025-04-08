# Team Underwater Perception for Event Response (TUPER)

This is the repository for the control section of the TUPER project. 

## Installation

### Prerequisites
Create a ROS2 workspace if you don't have one already:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone --recursive https://github.com/lucvt001/tuper.git
cd ..
```

### Build docker image

You should be at the root of the workspace for things to work properly.

```bash
docker build -t tuper:latest -f src/tuper/docker/Dockerfile .
```

### Run docker container

The typical practice is that the container be destroyed when it exits. However, for ease of development, we will keep it running even after we exit it, so that if you have to install some package or something, you don't have to do it every time you start the container.

You should be at the root of the workspace for volumes to mount correctly.

```bash
# Start the container
. src/tuper/docker/run_container.sh

# you can Ctrl-D to exit the container
# Do not close this terminal if you want to use the container!
```

Upon exiting the container, you can always resume it with the following command:

```bash
docker exec -it tuper bash
```

Remember to build the workspace (ignore a few packages that are only needed for simulation):

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-ignore sam_thruster_relay tuper_sim_utils     
```

