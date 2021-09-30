#!/bin/bash

red=`tput setaf 1`
green=`tput setaf 2`
yellow=`tput setaf 3`
blue=`tput setaf 4`
magenta=`tput setaf 5`
cyan=`tput setaf 6`
reset=`tput sgr0`

echo "${green}Installing ROS Desktop Full Version${reset}"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
echo "${green}Finished Installing ROS Desktop Full Version${reset}"

echo "${green}Setting up TurtleBot Dependencies${reset}"
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator
sudo apt-get install ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs
echo "${green}Finished Installing TurtleBot Dependencies${reset}"

echo "${green} Installing Arm Dependencies${reset}"
sudo apt install python-pip
sudo pip install modern_robotics
sudo apt-get install ros-kinetic-joint-state-publisher-gui
sudo apt-get install ros-kinetic-controller-manager
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-joint-trajectory-controller
sudo apt-get install ros-kinetic-moveit-commander
sudo apt-get install ros-kinetic-moveit-kinetic-tools
sudo apt-get install ros-kinetic-dynamixel-workbench-toolbox
