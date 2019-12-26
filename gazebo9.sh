#!/bin/bash

# include files
sudo cp -r ~/Desktop/ros/kinetic/include/gazebo_msgs /opt/ros/kinetic/include/
sudo cp -r ~/Desktop/ros/kinetic/include/gazebo_plugins/ /opt/ros/kinetic/include/
sudo cp -r ~/Desktop/ros/kinetic/include/gazebo_ros/ /opt/ros/kinetic/include/
sudo cp  ~/Desktop/ros/kinetic/include/gazebo_* /opt/ros/kinetic/include/

#lib files
sudo cp  ~/Desktop/ros/kinetic/lib/gazebo_* /opt/ros/kinetic/lib/
sudo cp -r  ~/Desktop/ros/kinetic/lib/gazebo_ros /opt/ros/kinetic/lib/
sudo cp  -r ~/Desktop/ros/kinetic/lib/gazebo_plugins /opt/ros/kinetic/lib/
sudo cp ~/Desktop/ros/kinetic/lib/libgazebo_* /opt/ros/kinetic/lib/
sudo cp -r ~/Desktop/ros/kinetic/lib/python2.7/dist-packages/gazebo_ros /opt/ros/kinetic/lib/python2.7/dist-packages/

# share files
sudo cp ~/Desktop/ros/kinetic/share/gazebo_* /opt/ros/kinetic/share/
sudo cp -r ~/Desktop/ros/kinetic/share/gazebo_dev /opt/ros/kinetic/share/
sudo cp -r ~/Desktop/ros/kinetic/share/gazebo_msgs /opt/ros/kinetic/share/
sudo cp -r ~/Desktop/ros/kinetic/share/gazebo_plugins /opt/ros/kinetic/share/
sudo cp -r ~/Desktop/ros/kinetic/share/gazebo_ros /opt/ros/kinetic/share/
sudo cp -r ~/Desktop/ros/kinetic/share/gazebo_ros_control /opt/ros/kinetic/share/
sudo cp -r ~/Desktop/ros/kinetic/share/gazebo_ros_pkgs /opt/ros/kinetic/share/

# rosdep update
rosdep update

