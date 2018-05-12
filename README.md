[![Build Status](https://travis-ci.com/gsilano/teamsannio_med_control.svg?token=j5Gz4tcDJ28z8njKZCzL&branch=master)](https://travis-ci.com/gsilano/teamsannio_med_control)

# teamsannio_med_control

The repository contains the teamsannio controller. Unfortunately, such controller is not able to compensate the wind gust and the sensors noise. Neverthless, we decided to share our code and to take part to the challenge submitting the developed control system.

Thanks to all the staff for the exciting challenge. It allowed us to understand, to learn and to work with new tools very popular within the UAV field.

Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
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
# Actions needed to run the code

# Running the code

To run each task, you have to use the launch files provided in the med_uav_decription/launch folder. Here is an example of how you can start task 1:

   ```
   $ roslaunch teamsannio_med_uav task1_world1.launch
   ```

Here are described some actions needed to run the code and reproduce the video results.


