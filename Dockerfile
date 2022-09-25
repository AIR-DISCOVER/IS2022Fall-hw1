FROM ros:noetic-ros-core-focal

RUN sed -i "s@http://.*archive.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list && \
     sed -i "s@http://.*security.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list
RUN apt-get update && \
     apt-get install -y --no-install-recommends \
     ros-noetic-catkin cmake gcc g++ make && \
     rm -rf /var/lib/apt/lists/* && \
     apt-get clean 

WORKDIR /opt
ADD course_ws /opt/course_ws
WORKDIR /opt/course_ws

RUN /opt/ros/noetic/env.sh catkin_make
CMD devel/env.sh rosrun MeArm MecArm.py