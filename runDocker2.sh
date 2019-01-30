#!/bin/bash
nvidia-docker run -it --rm \
        --security-opt seccomp=unconfined \
        --device=/dev/dri:/dev/dri \
        -e DISPLAY=$DISPLAY \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -v /home/kno/Documentos/Master/TFM/TFM/catkin_ws:/root/catkin_ws \
        -device=/dev/input/js0:/dev/input/js0 \
        --net host \
        --name ros2 \
        --env QT_X11_NO_MITSHM=1 \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        linsms/tfm:0.2
