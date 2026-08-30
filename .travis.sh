#!/bin/bash

set -ex

apt-get update -qq && apt-get install -y -q curl wget sudo lsb-release gnupg git sed build-essential ca-certificates # for docker
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"

# Install ROS
if [[ "$ROS_DISTRO" ==  "one" ]]; then
    # Configure ROS-O apt repository
    sudo mkdir -p /etc/apt/keyrings
    sudo curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs)-testing main" | sudo tee /etc/apt/sources.list.d/ros1.list
    sudo apt-get update -qq

    # Install and setup rosdep
    # Do not install python3-rosdep2, which is an outdated version of rosdep shipped via the Ubuntu repositories (instead of ROS)!
    sudo apt-get install -y -q python3-rosdep
    sudo rosdep init

    # Define custom rosdep package mapping
    echo "yaml https://ros.packages.techfak.net/ros-one.yaml one" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list
    rosdep update

    sudo apt-get install -y -q ros-one-desktop python3-catkin-tools python3-vcstool python-is-python3
else
    sudo sh -c "echo \"deb ${REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get update -qq

    if [[ "$ROS_DISTRO" ==  "noetic" ]]; then
        sudo apt-get install -y -q python3-catkin-pkg python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall-generator python3-osrf-pycommon python-is-python3
    else
        sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator
    fi
    sudo apt-get install -y -q ros-$ROS_DISTRO-catkin

    # Setup for rosdep
    sudo rosdep init
    rosdep update --include-eol-distros
fi

source /opt/ros/${ROS_DISTRO}/setup.bash

# Install source code
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/${REPOSITORY_NAME}
rosdep install --from-paths src -y -q -r --ignore-src --rosdistro ${ROS_DISTRO} # -r is indisapensible

# Build
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build -p1 -j1 --no-status
