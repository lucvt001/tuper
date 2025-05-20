## Installation

### Prerequisites
Create a ROS2 workspace if you don't have one already:

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src

git clone --recursive https://github.com/lucvt001/tuper.git
cd ..
```

Checkout to main branch for all submodules so that if you make some changes, you can push easily (run this inside `src/tuper`):
```bash
git submodule foreach --recursive 'git checkout main || echo "Submodule $name has no main branch"'
```

### Build docker image

You should be at the root of the workspace for things to work properly.

```bash
docker build -t tuper:latest -f src/tuper/docker/Dockerfile .
```

#### On RPI

```bash
docker pull lucvt001/tuper:latest
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
docker exec -it tuper_container bash
```

### Build the workspace

Remember to build the workspace (ignore a few packages that are only needed for simulation):

```bash
cd ~/colcon_ws

# On real vehicles
colcon build --executor sequential --symlink-install --packages-ignore sam_thruster_relay tuper_sim_utils monitoring

# On your own laptop (you must also install smarc2 repo)
colcon build --symlink-install
```

> Note: If you are working on a resource-constraint platform like RPI, you may have to increase the swap size to avoid out of memory errors. You can do this by editing the file `/etc/dphys-swapfile` and changing the value of `CONF_SWAPSIZE` to a larger number (e.g. 500-1000). Reboot to take effect. During the build process, open another terminal and run `htop` to monitor cpu+ram usage. If this still causes freezing, consider running `export MAKEFLAGS="-j 1"` (can be 2, 3, 4) to reduce the number of cores used (at the expense of build speed). As an initial estimate, try swapsize 500 and `MAKEFLAGS="-j 4"` and reboot.

>> Warning: After build, you must reduce the `CONF_SWAPSIZE` back to its original value (e.g. 100-300) and reboot again. This is because swap memory is meant to buy time and is extremely slow.

#### On RPI
```bash
git clone https://github.com/lucvt001/tuper_build.git
```

### Install additional packages

Only on your laptop (not on the real vehicle):
```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

Shortcut to get into the container:
```bash
echo "docker start tuper_container && docker exec -it tuper_container bash" >> ~/start_tuper_container.sh
```

```bash
. ~/start_tuper_container.sh
```