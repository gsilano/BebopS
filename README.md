[![Build Status](https://travis-ci.com/gsilano/BebopS.svg?token=j5Gz4tcDJ28z8njKZCzL&branch=master)](https://travis-ci.com/gsilano/BebopS)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg?style=flat-square)](https://www.firsttimersonly.com/)

# BebopS

BebopS is an extension of the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to modeling, developing and integrating the [Parrot Bebop 2](https://www.parrot.com/us/drones/parrot-bebop-2) quadcopter both in the physics based simulation environment Gazebo and in the software-in-the-loop (SIL) platform [Sphinx](http://www.sphinx-doc.org/en/master/).  

The repository was made for designing complex control systems for the Parrot Bebop, but it can also used for any other aircraft. Indeed, the controller implementation is a not easy process and having a complete software platform for simulating the multirotor behavior, considering also its on-board sensors, could give advantages in terms of coding and deployment of the controller software. 

Moreover, the software platform allows to detect and manage instabilities of the Parrot Bebop 2 that otherwise might not arise when considering only its Matlab/Simulink simulations. Finally, implementation details like synchronization, timing issues, fixed-point computation, overflow, divisions-by-zero, can be isolated when looking at the Matlab/Simulink platform and their effects can be investigated by considering the proposed SIL simulation platform.

The developed cose has been realesed as open-source under Apache 2.0 license. 

Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" 
 > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink 
 python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
 $ sudo rosdep init
 $ rosdep update
 $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 $ source ~/.bashrc
 $ sudo apt-get install python-rosinstall pythonrosinstall-generator python-wstool build-essential
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ catkin init
 $ git clone git@github.com:larics/rotors_simulator.git
 $ git clone git@github.com:larics/mav_comm.git
 $ git clone git@github.com:larics/BebopS.git
 $ cd ~/catkin_ws/src/rotors_simulator & git checkout med2018
 $ cd ~/catkin_ws/src/mav_comm & git checkout med2018
 $ rosdep update
 ```

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ rosdept install --from-paths serc -i
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Basic Usage
---------------------------------------------------------

Launching the simulation is quite simple, so as customizing it: it is enough to run in a terminal the command

   ```
   $ roslaunch bebops task1_world1.launch
   ```

Bugs & Feature Requests
--------------------------

Please report bugs and request features by using the [Issue Tracker](https://github.com/gsilano/BebopS/issues). Furthermore, please see the [Contributing.md](https://github.com/gsilano/BebopS/blob/master/CONTRIBUTING.md) file if you plan to help us to improve ROS package features.

YouTube video
---------------------------------------------------------
A YouTube video showing as the platform works is reported. The chosen sampling time (0.01ms) makes the simulation very slow. Indeed, to simulate only 10 seconds (`Sim Time` box in GAZEBO), 1 hour (`Real Time` box) is needed. Thus, the video has been speeded up at thirty-two times. Such behavior is determined by numerical issues occurring when the Euler's method with the 0.01ms sampling time is employed. It may be due to the iterative method (forward Euler) used to approximate the solution of the differential equations describing the aircraft dynamics. 

A waypoint composed by unit components along the axes and the yaw angle equal to zero has been used to test the control system performance. As is shown at the end of the video, the waypoint is reached with a very small error. The RotorS hovering example node has been used to this aim replacing the coordinates in the 3D vector `desired_position` and modifying the Gazebo GUI show up time from 5s to 0.1s.

[![MED18_Industrial_Challenge, teamsannio proposal](https://github.com/gsilano/teamsannio_med_control/wiki/images/Miniature_YouTube_MED18_Industrial_Challenge_Proposal.png)](https://youtu.be/BvsEA0zH7bU "MED18_Industrial_Challenge, teamsannio proposal")


