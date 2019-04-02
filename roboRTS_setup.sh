#!bin/bash

# Script to setup your system to use RoboRTS.

# RoboRTS is the framework we are using to build our software for the IRCA competition. 

# As of the making of this script, RoboRTS uses ROS Kinetic which only
# supports Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8)

# Install ROS Kinetic ----------------------------------------------------------

# Add ROS PPA
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
						> /etc/apt/sources.list.d/ros-latest.list'
						
# Setup keys for communicating with package server
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
								 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
								 
# Update package index and install ROS
sudo apt-get update
sudo apt-get install -y ros-kinetic-desktop-full

# Install tools and dependencies-----------------------------------

# The following are required for roboRTS
# see https://robomaster.github.io/RoboRTS-Tutorial/#/en/quick_start/setup_on_manifold2?id=software-dependency-configuration
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
                        libgoogle-glog-dev
                        
# The following are recommended by ROS
# see http://wiki.ros.org/kinetic/Installation/Ubuntu
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool

# Initialize rosdep (required to use ros)
sudo rosdep init
rosdep update

# Automatically load ROS enviroment variables when starting shell session
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Build the source code --------------------------------------------------------

# Create a workspace folder for ros
cd ..
mkdir -p ros_ws/src

# move source code
mv -t ./ros_ws/src/ ./IRCA2019/

# Update repository
cd ./ros_ws/src/IRCA2019/
git pull

# Compile code
cd ..
catkin_make

# Load enviroment variables
source devel/setup.bash
 
