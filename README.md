[![Build Status](https://travis-ci.com/gsilano/BebopS.svg?token=j5Gz4tcDJ28z8njKZCzL&branch=master)](https://travis-ci.com/gsilano/BebopS)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg?style=flat-square)](https://www.firsttimersonly.com/)

# BebopS

BebopS is an extension of the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to modeling, developing and integrating the [Parrot Bebop 2](https://www.parrot.com/us/drones/parrot-bebop-2) quadcopter in the physics based simulation environment Gazebo. The contribution can be also considered as a reference guide for expanding the RotorS functionalities in the Unmanned Aerial Vehicles (UAVs) filed and for simulating in a way rather close to reality the real aircraft behavior.

The repository was made for designing complex control systems for the Parrot Bebop 2, but it can also be used as a basis for any other aircraft with the aim of helping in the controller implementation. Indeed, implementing the own control algorithm is a not easy process and having a complete software platform for simulating the multirotor behavior, considering also its on-board sensors and the secondary effects, could give advantages in terms of coding and deployment of the controller software.

Finally, the software platform allows to detect and manage instabilities of the Parrot Bebop 2 that otherwise might not arise when considering only its Matlab/Simulink simulations. Moreover, implementation details such as synchronization, overflow or any other software related issue, can be isolated when looking at the Matlab/Simulink platform only, but their effects can be investigated by considering the proposed repository.

Simple cases study are considered (`task1_world.launch` and `task2_world.launch`) in order to show how the package works and the validity of the employed dynamical model together the control architecture of the quadrotor.

The code is released under Apache license, thus making it available for scientific and educational activities.

The platform was developed using Ubuntu 16.04 and the Kinetic Kame version of ROS, but it is also fully compatible with Ubuntu 18.04 and the Melodic Morenia distribution of ROS. Although backwards compatibility is guarantee, i.e., the platform is fully compatible with Indigo Igloo version of ROS and Ubuntu 14.04, such configuration is not recommended since the ROS support is expected to be closed in April 2019.

Below we provide the instructions necessary for getting started. See [BebopS' wiki](https://github.com/gsilano/BebopS/wiki) for more instructions and examples.

If you are using this simulator for research purposes especially for your publication, please take a look at the [Publications page](https://github.com/gsilano/BebopS/wiki/Publications). The page contains the core papers and all related works (using the platform).

The authors are grateful to the [LARICS Lab](https://larics.fer.hr/larics), University of Zagreb, for the initial work done to make the platform compatible with the latest changes to the RotorS package. Here the link to them [sofware repositry](https://github.com/larics/mmuav_gazebo). The software platform is an extract of the work carried out for the industrial challenge of the 26th Mediterranean Conference on Control and Automation (MED’18) in which the authors took part ([here the link](https://ieeexplore.ieee.org/document/8667511) to the conference report).

Installation Instructions - Ubuntu 18.04 with ROS Melodic and Gazebo 9
---------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink
$ sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential
```

2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b med18_gazebo9 https://github.com/gsilano/rotors_simulator.git
$ git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
$ git clone -b dev/gazebo9 https://github.com/gsilano/BebopS.git
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git
$ cd ~/catkin_ws
```

3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ catkin build
```

4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

5. Update the pre-installed Gazebo version. This fix the issue with the `error in REST request for accessing api.ignition.org`

```console
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt update
$ sudo apt install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

> In the event that the simulation does not start, the problem may be related to Gazebo and missing packages. Therefore, run the following commands. More details are reported in [#25](https://github.com/gsilano/CrazyS/issues/25).
```console
$ sudo apt-get remove ros-melodic-gazebo* gazebo*
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9 gazebo9-* ros-melodic-gazebo-*
$ sudo apt upgrade
```

Installation Instructions - Ubuntu 16.04 with ROS Kinetic and Gazebo 7
---------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

 1. Install and initialize ROS Kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

```console
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink
$ sudo apt-get install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python-rosinstall python-rosinstall-generator build-essential
```

 2. If you don't have ROS workspace yet you can do so by

```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/catkin_ws/
$ catkin init
$ cd ~/catkin_ws/src
$ git clone -b med18 https://github.com/gsilano/rotors_simulator.git
$ git clone -b med18 https://github.com/gsilano/mav_comm.git
$ git clone https://github.com/gsilano/BebopS.git
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git
```

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```console
$ rosdep install --from-paths src -i
$ catkin build
```

 4. Add sourcing to your `.bashrc` file

```console
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Installation Instructions - Ubuntu 16.04 with ROS Kinetic and Gazebo 9
---------------------------------------------------------
To use the code developed and stored in this repository with ROS Kinetic and Gazebo 9, first follow what is reported in the previous section. Then, use the instruction below.

 1. Remove Gazebo 7 and all related packages, and then install Gazebo 9:

```console
$ sudo apt-get remove ros-kinetic-gazebo* gazebo*
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo9 gazebo9-* ros-kinetic-gazebo9-*
$ sudo apt upgrade
```
> **Note** Remove `ros-kinetic-gazebo9-*` from the command `sudo apt-get install gazebo9 gazebo9-* ros-kinetic-gazebo9-*` if fetch problems should appear during the installation.

 2. Additional packages are required to build the package.

```console
$ sudo apt-get install libeigen3-dev ros-kinetic-image-view ros-kinetic-parrot-arsdk libprotobuf-dev libprotoc-dev ros-kinetic-joy-teleop ros-kinetic-nav-msgs ros-kinetic-mav-msgs libyaml-cpp-dev ros-kinetic-nodelet ros-kinetic-mav-planning-msgs ros-kinetic-urdf ros-kinetic-image-transport ros-kinetic-roslint ros-kinetic-angles ros-kinetic-cv-bridge ros-kinetic-tf2-geometry-msgs ros-kinetic-xacro ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev ros-kinetic-camera-info-manager ros-kinetic-cmake-modules ros-kinetic-gazebo-msgs ros-kinetic-mavros-msgs ros-kinetic-control-toolbox ros-kinetic-mav-msgs ros-kinetic-libmavconn ros-kinetic-mavros ros-kinetic-octomap-msgs ros-kinetic-geographic-msgs ros-kinetic-mavlink ros-kinetic-mavros-extras ros-kinetic-mav-planning-msgs ros-kinetic-joy
```
> **Note** Missing packages can be found and then installed by using the command `rosdep check --from-paths src` into the `catkin_ws` folder.

 3. Make Gazebo 9 compatible with ROS Kinetic Kame

```console
$ cd ~
$ mkdir -p ros-kinetic-gazebo9-pkgs
$ cd ros-kinetic-gazebo9-pkgs
$ git clone -b feature/ros-kinetic-gazebo9-pkgs https://github.com/gsilano/BebopS.git
$ cd BebopS
$ chmod 777 gazebo9.sh
$ ./gazebo9.sh
$ cd ~
$ sudo rm -rf ros-kinetic-gazebo9-pkgs # delete the folder after the installation
```

4. Clean the workspace and compile again the code

```console
$ cd ~/catkin_ws
$ catkin clean # digit y when required
$ cd ~/catkin_ws/src/rotors_simulator
$ git checkout med18_gazebo9
$ cd ~/catkin_ws/src/mav_comm
$ git checkout med18_gazebo9
$ cd ~/catkin_ws/src/BebopS
$ git checkout dev/gazebo9
$ cd ~/catkin_ws
$ catkin build
$ source ~/.bashrc
```

> **Note** In case the `ERROR[rotors_gazebo_plugins]` error is displayed, run the following commands
>```console
>$ sudo apt-get install ros-kinetic-gazebo9-plugins
>$ sudo apt-get install apt ros-kinetic-gazebo9-ros
>$ sudo apt-get install apt ros-kinetic-gazebo9-dev
>```

This guide can be used a basis for fixing what has been discussed in [ethz-asl/rotors_simulator#506](https://github.com/ethz-asl/rotors_simulator/pull/506).

Basic Usage
---------------------------------------------------------

Running the simulation is quite simple, so as customizing it: it is enough to run in a terminal the command

```console
$ roslaunch bebop_simulator bebop_without_controller.launch
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an on-line database. To avoid any problems when starting the simulation for the first time, you may run the `gazebo` command in the terminal line.

The `bebop_without_controller.launch` file simulates the Parrot Bebop dynamics when any controller is in the loop. Therefore, the drone pose can be modified by publishing the propellers angular velocity on the `/gazebo/command/motor_speed` topic.

> **Note** If you use the version compatible with ROS Melodic, `/command/motor_speed` is the topic to consider.

Moreover, external disturbances can also be simulated by varying the contents of the variables: `wind_force` (it represents the wind force expressed in Newton), `wind_start` (it indicates the time in seconds after which external forces will begin to act), `wind_duration` (the interval time), `wind_direction` (the wind direction along the x, y and z-axis, its values are bounded between [-1, 1]).

To fly the multirotor, it is necessary to generate thrust with the rotors, this is achieved by sending commands to the quadrotor that make the rotors spin.

 ```console
$ rostopic pub /gazebo/command/motor_speed mav_msgs/Actuators '{angular_velocities: [1000, 1000, 1000, 1000]}'
```

> **Note** If you use the ROS Melodic version of the code, the command to consider is
```console
$ rostopic pub /command/motor_speed mav_msgs/Actuators '{angular_velocities: [1000, 1000, 1000, 1000]}'
```

To speed up the simulation, a certain set of sensors can be included when simulating the drone dynamics by varying the flags: `enable_odometry_sensor_with_noise`/`disable_odometry_sensor_with_noise` (it includes the odometry sensor with bias and noise terms), `enable_ground_truth_sensor` (it enables the ground truth sensor), `enable_wind_plugin` (even external disturbances will be simulated), `enable_laser1D` (it enables the RotorS default IMU, i.e., the ADIS16448 IMU) and `enable_laser1D` (it enables the 1-D laser scanner).

These value can be modified before simulating the drone behavior acting on the launch file or at runtime by running on the terminal:

```console
$ roslaunch bebop_simulator bebop_without_controller.launch enable_odometry_sensor_with_noise:=true
```

Finally, the waypoint and the Kalman filters, as well as the data storage (file used as log), can be enabled/disabled by using the variables: `csvFilesStoring`, `csvFilesStoringTime` (simulation time after which the data will be saved), `user_account` (required to define the storage path), `waypoint_filter` and `EKFActive`.   

While, running in a terminal the command

```console
$ roslaunch bebop_simulator task1_world.launch
```

the Parrot Bebop takes off from the ground and keeps indefinitely the hovering position subjected to wind gusts (up to 0.5 N) for a minute. Conversely,

```console
$ roslaunch bebop_simulator task2_world.launch
```

the drone starts to follow the trajectory expressed as a sequence of waypoints (`x_r, y_r, z_r` and `\psi_r`) published at a settled time (`t_0, t_1, t_3`, etc.), as described in `waypoint.txt` file. To avoid system instabilities, a waypoint filter is employed to smooth the trajectory.

In the repository a simple configuration file for RVIZ (see #28) and multi-robot example (see #29) are provided. To run the simulation, just simply type these command in the terminal line.

```console
# To run a multi-robot scenario along with the control algorithm
$ roslaunch bebop_simulator multiple_task2_world.launch
# To run a multi-robot scenario without the position_controller
$ roslaunch bebop_simulator multiple_bebop_without_controller.launch
```

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionality, although the platform is fully compatible with the **Gazebo 9**. Before running the script, consider the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.

Bugs & Feature Requests
--------------------------

Please report bugs and request features by using the [Issue Tracker](https://github.com/gsilano/BebopS/issues). Furthermore, please see the [Contributing.md](https://github.com/gsilano/BebopS/blob/master/CONTRIBUTING.md) file if you plan to help us to improve ROS package features.

YouTube video
---------------------------------------------------------
A YouTube video showing the Parrot Bebop during the trajectory tracking is reported. The trajectory, described using waypoints, is shown in the `waypoint.txt` file.

[![MED18_Industrial_Challenge, BebopS](https://github.com/gsilano/BebopS/wiki/images/Video_Miniature_YouTube_SMC19.png)](https://youtu.be/ERkgSCoM6OI "MED18_Industrial_Challenge, BebopS")
