[![Build Status](https://travis-ci.com/gsilano/BebopS.svg?token=j5Gz4tcDJ28z8njKZCzL&branch=master)](https://travis-ci.com/gsilano/BebopS)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg?style=flat-square)](https://www.firsttimersonly.com/)

# BebopS

BebopS is an extension of the ROS package [RotorS](https://github.com/ethz-asl/rotors_simulator), aimed to modeling, developing and integrating the [Parrot Bebop 2](https://www.parrot.com/us/drones/parrot-bebop-2) quadcopter both in the physics based simulation environment Gazebo and in the software-in-the-loop (SIL) platform [Sphinx](http://www.sphinx-doc.org/en/master/). The contribution can be also considered as a reference guide for expanding the RotorS functionalities in the UAVs filed and for simulating in a way rather close to reality the real aircraft behavior. 

The repository was made for designing complex control systems for the Parrot Bebop, but it can also used for any other aircraft. Indeed, the controller implementation is a not easy process and having a complete software platform for simulating the multirotor behavior, considering also its on-board sensors, could give advantages in terms of coding and deployment of the controller software. 

Moreover, the software platform allows to detect and manage instabilities of the Parrot Bebop 2 that otherwise might not arise when considering only its Matlab/Simulink simulations. Finally, implementation details like synchronization, timing issues, fixed-point computation, overflow, divisions-by-zero, can be isolated when looking at the Matlab/Simulink platform and their effects can be investigated by considering the proposed SIL simulation platform.

Simple cases study are considered (`task1_world.launch` and `task2_world.launch`) in order to show how the package works and the validity of the employed dynamical model together the control architecture of the quadcopter.

The code is released under Apache license, thus making it available for scientific and educational activities.

The platform has been developed by using Ubuntu 16.04 and the Kinetic Kame version of ROS. Although the platform is fully compatible with Indigo Igloo version of ROS and Ubuntu 14.04, such configuration is not recommended since the ROS support will close in April 2019.

Below we provide the instructions necessary for getting started. See [BbopS' wiki](https://github.com/gsilano/BebopS/wiki) for more instructions and examples.

If you are using this simulator within the research for your publication, please take a look at the [Publications page](https://github.com/gsilano/BebopS/wiki/Publications). The page contains core papers and all linked works (using the platform).

Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

 1. Install and initialize ROS kinetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" 
 > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink 
 python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox 
 python-rosinstall python-rosinstall-generator build-essential
 $ sudo rosdep init
 $ rosdep update
 $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
 $ source ~/.bashrc
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
 $ cd ~/catkin_ws/src/rotors_simulator & git checkout med18
 $ cd ~/catkin_ws/src/mav_comm & git checkout med18
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
   $ roslaunch bebops bebop_without_controller.launch
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
   $ roslaunch bebopS bebop_without_controller.launch enable_odometry_sensor_with_noise:=true
   ```
   
Finally, the waypoint and Kalman filters, as well as the data storage, can be enabled/disabled by using the variables: `csvFilesStoring`, `csvFilesStoringTime` (simulation time after which the data will be saved), `user_account` (required to define the storage path), `waypoint_filter` and `EKFActive`.   

While, running in a terminal the command

   ```
   $ roslaunch bebopS task1_world.launch
   ```
   
the Parrot Bebop takes off from the ground and keeps indefinitely the hovering position subjected to wind gusts (up to 0.5 N) for a minute. Conversely, 

   ```
   $ roslaunch bebopS task2_world.launch
   ```
   
the drone starts to follow the trajectory expressed as a sequence of waypoints (x_r, y_r, z_r and \psi_r) published at a settled time (t_0, t_1, t_3, etc.), as described in `waypoint.txt` file. To avoid system instabilities, a waypoint filter is employed to smooth the trajectory.

Installation Instructions - Ubuntu 16.04 with ROS Kinetic and Sphinx
--------------------------------------------------------------------

Preliminary actions are required to use the code developed with the Parrot-Sphinx simulator. First of all, **what is Sphinx?** Sphinx is a simulation tool initially thought to cover the needs of Parrot engineers developing drone software. The main concept is to run a Parrot drone firmware on a PC, in an isolated environment well separated from the host system, while Gazebo is in charge of simulating the physical and visual surroundings of the drone. For further information on Sphinx, please take a look at the [Parrot for Developers website](https://developer.parrot.com/docs/sphinx/index.html).

Sphinx can run only on Linux 64 bits, and a minimum of 1 GByte of storage is necessary to its exection.

1. Add a new apt repository to the systema dn install the packages

```
$ echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null
$ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5
$ sudo apt-get update
$ sudo apt-get install parrot-sphinx
```
>Note: At the end of installation, a system log out is needed and then relog is needed to complete the installation.

2. Let's start the simulator.

For Linux distributions coming with [systemd](https://en.wikipedia.org/wiki/Systemd), Firmwared is installed as a systemd service. Therefore you just need to launch the service by entering the command:

```
$ sudo systemctl start firmwared.service
```

>Note: A system restart is required to able to run the firmware. For old Linux distribution, firmwared needs to be launched manually from a shell ` $ sudo firmwared `.

The execution of firmwared is blocking so do not close the shell. To check if all went well, enter the following command:

```
$ fdc ping
PONG
```
You should get a _PONG_ in response. Otherwise, check the [Troubleshooting page](https://developer.parrot.com/docs/sphinx/troubleshooting.html).

3. Check the wifi interface name

At this time, it is importanto to check whe wifi interface name by using the following command:

```
$ iwconfig
```

and remember the host wifi interface name (usualy "wlan0" or "wlx"). It will be necessary in the next step.

Simulated drones may use the host system wifi interface to communicate with the drone controller (e.g., FreeFlight). For his own purpose, a simulated drone "steals" the host wifi interface. While the simulation is running, the wifi interface is only accessible from within the simulated firmware. If an interface for Internet access is already on use, it would be disconnected.

4. Launch the first simulation by choosing a `.drone` file and start Sphinx with it. Several `.drone` files are provided along with Sphinx installation. [Here](https://developer.parrot.com/docs/sphinx/dronefile.html) there is a full list of supported devices.

```
$ sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone
```

before simulating, the stolen interface needs to be configured. For further details, take a look at the [Parrot-Sphinx documentation](https://developer.parrot.com/docs/sphinx/firststep.html). 

> Note: As it is the first time Sphinx is started with this `.drone` file, it may take several seconds to download the drone firmware from the external server. Once the firmware is loaded, the simulation starts.

5. At this time, the launch files implementing the hovering and trajectory tracking examples, respectively, can be executed by using the [**bebop_autonomy**](https://bebop-autonomy.readthedocs.io/en/latest/) ROS package. Below are the installation instructions for the ROS package:

```
$ sudo apt-get install build-essential python-rosdep python-catkin-tools
# Create and initialize the workspace
$ mkdir -p ~/bebop_ws/src && cd ~/bebop_ws
$ catkin init
$ git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
# Update rosdep database and install dependencies (including parrot_arsdk)
$ rosdep update
$ rosdep install --from-paths src -i
# Build the workspace
$ catkin build
```

Then, the access permissions for the files listed in the scripts folder have to be changed. It can be done, using the commands 

```
# To install the unbuffer command required for the script
$ sudo apt install expect
$ cd ~/catkin_ws/src/bebopS/scripts/
# Sh script to start recording data from the Parrot-Sphinx simulator
$ sudo chmod 777 data_logger.sh
# Awk script in charge of publishing the Parrot-Sphinx simulator data
$ sudo chmod 777 data_logger_publishing.awk
```

Finally, the simulation can be performed through the commands listed below (they have to be runned in three different terminals)

```
# A collection of nodes and programs that are pre-requisites of a ROS-based system
$ roscore
```

```
# Sh script to enable the publication of the data logger
$ rosrun bebopS data_logger.sh
```

```
# Hovering example
$ roslaunch bebopS task1_world_with_sphinx.launch
# Trajectory tracking example
$ roslaunch bebopS task2_world_with_sphinx.launch
```

Bugs & Feature Requests
--------------------------

Please report bugs and request features by using the [Issue Tracker](https://github.com/gsilano/BebopS/issues). Furthermore, please see the [Contributing.md](https://github.com/gsilano/BebopS/blob/master/CONTRIBUTING.md) file if you plan to help us to improve ROS package features.

YouTube video
---------------------------------------------------------
A YouTube video showing the Parrot Bebop during the trajectory tracking is reported. The trajectory, described using waypoints, is shown in the `waypoint.txt` file.

[![MED18_Industrial_Challenge, BebopS](https://github.com/gsilano/BebopS/wiki/images/Video_Miniature_YouTube_ECC19.png)](https://youtu.be/ERkgSCoM6OI "MED18_Industrial_Challenge, BebopS")


