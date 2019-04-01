#!bin/bash

# Script to setup a system to use RoboRTS. It will also clone and compile the 
# current version of our source code.

# RoboRTS is the framework we are using to build our software for the IRCA competition. 

# As of the making of this script, robot_RTS uses ROS Kinetic which is only
# supports Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8)


# Install dependencies
sudo apt-get update

sudo apt-get install -y ros-kinetic-opencv3             \
                        ros-kinetic-cv-bridge           \
                        ros-kinetic-image-transport     \
                        ros-kinetic-stage-ros           \
                        ros-kinetic-map-server          \
                        ros-kinetic-laser-geometry      \
                        ros-kinetic-interactive-markers \
                        ros-kinetic-tf                  \
                        ros-kinetic-pcl-*               \
                        ros-kinetic-libg2o              \
                        ros-kinetic-rplidar-ros         \
                        ros-kinetic-rviz                \
                        protobuf-compiler               \
                        libprotobuf-dev                 \
                        libsuitesparse-dev              \
                        libgoogle-glog-dev              \

# Clone our source code
git clone https://github.com/robogrinder/IRCA2019.git
cd ./IRCA2019
# Compile code
catkin_make
# Load enviroment variables
source devel/setup.bash
 
