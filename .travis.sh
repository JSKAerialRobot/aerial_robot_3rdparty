#!/bin/bash

set -ex

apt-get update -qq && apt-get install -y -q wget sudo lsb-release gnupg git sed build-essential # for docker
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo sh -c "echo \"deb ${REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq

# Install ROS
if [[ "$ROS_DISTRO" ==  "noetic" ]]; then
    sudo apt-get install -y -q python3-catkin-pkg python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall-generator python3-osrf-pycommon python-is-python3
else
    sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator
fi
sudo apt-get install -y -q ros-$ROS_DISTRO-catkin

source /opt/ros/${ROS_DISTRO}/setup.bash

# Setup for rosdep
sudo rosdep init
# use snapshot of rosdep list
    # https://github.com/ros/rosdistro/pull/31570#issuecomment-1000497517
if [[ "$ROS_DISTRO" = "kinetic" ]]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    sudo wget https://gist.githubusercontent.com/cottsay/b27a46e53b8f7453bf9ff637d32ea283/raw/476b3714bb90cfbc6b8b9d068162fc6408fa7f76/30-xenial.list -O /etc/ros/rosdep/sources.list.d/30-xenial.list
fi
rosdep update --include-eol-distros

# Install source code
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/${REPOSITORY_NAME}
wstool init src
rosdep install --from-paths src -y -q -r --ignore-src --rosdistro ${ROS_DISTRO} # -r is indisapensible

# Build
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build -p1 -j1 --no-status
