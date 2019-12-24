[![Build Status](https://travis-ci.com/gsilano/BebopS.svg?token=j5Gz4tcDJ28z8njKZCzL&branch=master)](https://travis-ci.com/gsilano/BebopS)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg?style=flat-square)](https://www.firsttimersonly.com/)

# BebopS

BebopS is an extension of the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to modeling, developing and integrating the [Parrot Bebop 2](https://www.parrot.com/us/drones/parrot-bebop-2) quadcopter in the physics based simulation environment Gazebo. The contribution can be also considered as a reference guide for expanding the RotorS functionalities in the UAVs filed and for simulating in a way rather close to reality the real aircraft behavior. 

The repository was made for designing complex control systems for the Parrot Bebop, but it can also used for any other aircraft. Indeed, the controller implementation is a not easy process and having a complete software platform for simulating the multirotor behavior, considering also its on-board sensors, could give advantages in terms of coding and deployment of the controller software. 

Moreover, the software platform allows to detect and manage instabilities of the Parrot Bebop 2 that otherwise might not arise when considering only its Matlab/Simulink simulations. Finally, implementation details synchronization, overflow or any other software realted issue, can be isolated when looking at the Matlab/Simulink platform only, but their effects can be investigated by considering the proposed repository.

Simple cases study are considered (`task1_world.launch` and `task2_world.launch`) in order to show how the package works and the validity of the employed dynamical model together the control architecture of the quadcopter.

The code is released under Apache license, thus making it available for scientific and educational activities.

The platform has been developed by using Ubuntu 16.04 and the Kinetic Kame version of ROS. Although the platform is fully compatible with Indigo Igloo version of ROS and Ubuntu 14.04, such configuration is not recommended since the ROS support is expected to be closed in April 2019.

Below we provide the instructions necessary for getting started. See [BebopS' wiki](https://github.com/gsilano/BebopS/wiki) for more instructions and examples.

If you are using this simulator within the research for your publication, please take a look at the [Publications page](https://github.com/gsilano/BebopS/wiki/Publications). The page contains core papers and all linked works (using the platform).

The authors are grateful to the [LARICS Lab](https://larics.fer.hr/larics) of the University of Zagreb that created the basis for the ROS package through the [sofware repositry](https://github.com/larics/mmuav_gazebo). In fact, the platform is an extract of the work carried out for the industrial challenge of the 26th Mediterranean Conference on Control and Automation (MED’18) in which the authors took part ([here the link](https://ieeexplore.ieee.org/document/8667511) to the conference report).

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
 $ sudo apt-get install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
 $ sudo rosdep init
 $ rosdep update
 $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 $ source ~/.bashrc
 $ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ cd ~/catkin_ws/
 $ catkin init
 $ cd ~/catkin_ws/src
 $ git clone https://github.com/gsilano/rotors_simulator.git
 $ git clone https://github.com/gsilano/mav_comm
 $ git clone https://github.com/gsilano/BebopS.git
 $ git clone https://github.com/AutonomyLab/bebop_autonomy.git
 $ cd ~/catkin_ws/src/rotors_simulator 
 $ git checkout med18
 $ cd ~/catkin_ws/src/mav_comm
 $ git checkout med18
 $ cd ~/catkin_ws
 $ rosdep update
 ```

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ rosdep install --from-paths src -i
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
   $ roslaunch bebop_simulator bebop_without_controller.launch
   ```
   
> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database. To avoid any problems when starting the simulation for the first time, you may run the `gazebo` command in the terminal line.

The `bebop_without_controller.launch` file lets simulate the Parrot Bebop dynamics when no controllers are in the loop. Therefore, the drone pose can be modified by publishing the propellers angular velocity on the `/gazebo/command/motor_speed` topic. Moreover, external disturbances can also be simulated by varying the contents of the variables: `wind_force` (it represents the wind force expressed in Newton), `wind_start` (it indicates the time in seconds after which external forces will begin to act), `wind_duration` (the inveral time), `wind_direction` (the wind direction along the x, y and z-axis, its values are bounded between [-1, 1]). 

To let the multicopter fly you need to generate thrust with the rotors, this is achieved by sending commands to the multicopter, which make the rotors spin. 

   ```
   $ rostopic pub /gazebo/command/motor_speed mav_msgs/Actuators '{angular_velocities: [1000, 1000, 1000, 1000]}'
   ```

To speed up the simulation, a certain set of sensors can be included when simulating the drone dynamics by varying the flags: `enable_odometry_sensor_with_noise`/`disable_odometry_sensor_with_noise` (it includes the odometry sensor with bias and noise terms), `enable_ground_truth_sensor` (it enables the ground truth sensor), `enable_wind_plugin` (even external disturbances will be simulated) and `enable_laser1D` (it enables the 1-D laser scanner).

These value can be modified before simulating the drone behavior acting on the launch file or at runtime by running on the terminal:

   ```
   $ roslaunch bebop_simulator bebop_without_controller.launch enable_odometry_sensor_with_noise:=true
   ```
   
Finally, the waypoint and Kalman filters, as well as the data storage, can be enabled/disabled by using the variables: `csvFilesStoring`, `csvFilesStoringTime` (simulation time after which the data will be saved), `user_account` (required to define the storage path), `waypoint_filter` and `EKFActive`.   

While, running in a terminal the command

   ```
   $ roslaunch bebop_simulator task1_world.launch
   ```
   
the Parrot Bebop takes off from the ground and keeps indefinitely the hovering position subjected to wind gusts (up to 0.5 N) for a minute. Conversely, 

   ```
   $ roslaunch bebop_simulator task2_world.launch
   ```
   
the drone starts to follow the trajectory expressed as a sequence of waypoints (x_r, y_r, z_r and \psi_r) published at a settled time (t_0, t_1, t_3, etc.), as described in `waypoint.txt` file. To avoid system instabilities, a waypoint filter is employed to smooth the trajectory.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, although the platform is fully compatible with the Gazebo 9. Before running the script, consider the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

ROS Kinetic installs Gazebo 7 by default so it is recommended to remove the previous installation. Proceeded to the installation section if installing for the first time.

```
$ sudo apt-get remove ros-kinetic-gazebo*
$ sudo apt-get upgrade
```

Install the core of the ROS and rest of the packages can be added manually

```
$ sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control ros-kinetic-gazebo9* (for Gazebo 9)
```

Bugs & Feature Requests
--------------------------

Please report bugs and request features by using the [Issue Tracker](https://github.com/gsilano/BebopS/issues). Furthermore, please see the [Contributing.md](https://github.com/gsilano/BebopS/blob/master/CONTRIBUTING.md) file if you plan to help us to improve ROS package features.

YouTube video
---------------------------------------------------------
A YouTube video showing the Parrot Bebop during the trajectory tracking is reported. The trajectory, described using waypoints, is shown in the `waypoint.txt` file.

[![MED18_Industrial_Challenge, BebopS](https://github.com/gsilano/BebopS/wiki/images/Video_Miniature_YouTube_SMC19.png)](https://youtu.be/ERkgSCoM6OI "MED18_Industrial_Challenge, BebopS")


