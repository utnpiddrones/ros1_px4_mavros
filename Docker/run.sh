#!/bin/bash

image="utnpiddrones/ros1_px4_mavros"
xhost + &>/dev/null

if ! docker image ls | grep "${image}" &>/dev/null; then
    docker pull "${image}"
fi

docker run -i -t --rm --privileged \
    --name "cont" \
    -e DISPLAY=${DISPLAY} \
    -e LOCAL_USER_ID="$(id -u)" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/../my_mavros:/home/user/catkin_my_mavros/src/my_mavros" \
    -v "$(pwd)/my_entrypoint.sh:/my_entrypoint.sh" \
    -v "$(pwd)/init.sh:/init.sh" \
    "${image}" \
    "sleep 5d"
