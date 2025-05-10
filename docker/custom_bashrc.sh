unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /opt/ros/humble/setup.bash
source $HOME/colcon_ws/install/setup.bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{name}]: {message}"