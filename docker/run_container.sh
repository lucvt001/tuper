docker run -it --privileged \
    --name tuper_container \
    --network=host \
    --ipc=host \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    -v $PWD:/home/smarc2user/colcon_ws \
    luc001/tuper:latest
