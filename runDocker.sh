#!/bin/bash
nvidia-docker run -it --rm -e DISPLAY=:0 -v $HOME/.Xauthority:/root/.Xauthority -v /home/kno/Documentos/Master/TFM/TFM/catkin_ws:/root/catkin_ws -device=/dev/input/js0:/dev/input/js0 --net host --name ros --env QT_X11_NO_MITSHM=1 linsms/gazebo-ros-indigo-ardrone:working
