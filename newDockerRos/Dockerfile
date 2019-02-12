FROM nvidia/cuda:9.0-cudnn7-runtime

# install bootstrap tools
RUN apt-get update && apt-get upgrade -y && \
  apt-get install --no-install-recommends -y \
  lsb-release && \
  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
  apt-get update && \
  apt-get install python-pip -y && \
  apt-get install ros-kinetic-desktop-full -y 

RUN apt install ros-kinetic-ardrone-autonomy git subversion cmake g++ -y && apt-get autoremove -y

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && \
  rosdep init && \
  rosdep update"
