docker run -it \
    --name tuper_container \
    --network=host \
    --ipc=host \
    -v /dev/shm:/dev/shm \
    -v $PWD:/home/smarc2user/colcon_ws \
    tuper:latest