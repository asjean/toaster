#!/bin/bash
#Script to install packages before attempting to build if necessary

rosdep install --from-paths src --ignore-src -r -y
#sudo apt install ros-noetic-teleop-twist-keyboard
