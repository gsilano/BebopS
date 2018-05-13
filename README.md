[![Build Status](https://travis-ci.com/gsilano/teamsannio_med_control.svg?token=j5Gz4tcDJ28z8njKZCzL&branch=master)](https://travis-ci.com/gsilano/teamsannio_med_control)

# teamsannio_med_control

The repository contains the teamsannio controller. Unfortunately, such controller is not able to compensate the wind gust and the sensors' noise. Nevertheless, we decided to share our code and to take part in the challenge submitting the developed control system.

Thanks to all the staff for the exciting challenge. It allowed us to understand, to learn and to work with new tools very popular within the UAV field.

Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/kinetic/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ git clone git@github.com:larics/rotors_simulator.git
 $ git clone git@github.com:larics/mav_comm.git
 $ git clone git@github.com:larics/med_uav_description.git
 $ git clone git@github.com:larics/teamsannio_med_control.git
 $ cd ~/catkin_ws/src/rotors_simulator & git checkout med2018
 $ cd ~/catkin_ws/src/mav_comm & git checkout med2018
 ```

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```
Actions needed to run the code
---------------------------------------------------------

The RotorS basic world (`rotors_gazebo/worlds/basic.world`) has been used as the empty world for the simulation environment. In such world, the `max_step_size` and `real_time_update_rate` have been modified to take into account the fixed step size used in MATLAB to simulate the entire system: the Bebop model with the controller in the loop. In this way, we were able to move the sampling time from 1ms to 0.01ms.

Furthermore, the sensors noise in the `med_uav_description/urd/bebop.urdf.xacro` file has been removed. Below the new (`+` symbol) and the previous version (`-` symbol) of the changed row lines are reported.

   ```
   -		noise_normal_position="0.01 0.01 0.01"
   -		noise_normal_quaternion="0.017 0.017 0.017"
   -		noise_normal_linear_velocity="0.01 0.01 0.01"
   -		noise_normal_angular_velocity="0.01 0.01 0.01"
   +		noise_normal_position="0.0 0.0 0.0"
   +		noise_normal_quaternion="0.0 0.0 0.0"
   +		noise_normal_linear_velocity="0.0 0.0 0.0"
   +		noise_normal_angular_velocity="0.0 0.0 0.0"
   ```

Running the code
---------------------------------------------------------

To run the developed task 1, you have to use the launch file provided in the `teamsannio_med_control/launch` folder. Here is an example of how you can start task 1:

   ```
   $ roslaunch teamsannio_med_uav task1_world1.launch
   ```


YouTube videos
---------------------------------------------------------
A YouTube video showing as the platform works is reported. The chosen sampling time (0.01ms) makes the simulation very slow. Indeed, to simulate only 10 seconds (`Sim Time` box in GAZEBO), 1 hour (`Real Time` box) is needed. Thus, the video has been speeded up at thirty-two times. Such behavior is determined by numerical issues occurring when the Euler's method with the 0.01ms sampling time is employed. It may be due to the iterative method (forward Euler) used to approximate the solution of the differential equations describing the aircraft dynamics. 

A waypoint composed by unit components along the axes and the yaw angle equal to zero has been used to test the control system performance. As is shown at the end of the video, the waypoint is reached with a very small error. The RotorS hovering example node has been used to this aim replacing the coordinates in the 3D vector `desired_position` and modifying the Gazebo GUI show up time from 5s to 0.1s.

[![MED18_Industrial_Challenge, teamsannio proposal](https://github.com/gsilano/teamsannio_med_control/wiki/images/Miniature_YouTube_MED18_Industrial_Challenge_Proposal.png)](https://youtu.be/GzA3hle3lZU "MED18_Industrial_Challenge, teamsannio proposal")


