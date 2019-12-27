^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package BebopS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2019-12-27)
------------------
* Updated the README.md file with instructions for Ubuntu 16.04, ROS Kinetic Kame and Gazebo 9 (master)
* Contributors: Giuseppe Silano

2.0.1 (2019-12-26)
------------------
* Updated the README.md file with instructions for Ubuntu 18.04, ROS Melodic Morenia and Gazebo 9 (master)
* Deleted the README.md file to avoid misunderstanding in the installation procedure (dev/gazebo9 branch)
* Inserted the default RotorS IMU (ADIS16448 IMU) in the list of the available sensors when running the simulation
* Disable shadows in Gazebo 9 and added the rotors_interface_plugin. The plugin is necessary for reading the sensors messages
* Deleted TravisCI (.travisci.yml) file. Only one file is present in the master. The file handles all the distro.
* Contributors: Giuseppe Silano

0.2.2 (2019-12-26)
------------------
* Inserted the default RotorS IMU (ADIS16448 IMU) in the list of the available sensors when running the simulation
* Disabled shadows in Gazebo 9 and Gazebo 7
* Contributors: Giuseppe Silano

0.2.1 (2019-04-04)
------------------
* The namespace was modified considering the ROS rules (only lowercase letters)
* Improved the code quality and typos in the README.md file
* Contributors: Giuseppe Silano

0.2.0 (2019-01-31)
------------------
* The package has been renamed considering the aim of the repository. Therefore, the CMake and include files have been modified based on these changes.
* The files contained into the med_uav_description folder have been moved to the BebopS repository
* The controller architecture, which follows the position_controller.cpp library, has been modified as described in the ECC'19 paper. For more information on how it works, please take a look at the publications section into the Wiki section.
* When simulating the Parrot Bebop 2 that follows a path along the scenario (it is described through a sequence of waypoints), it is possible to decide which sensor will be simulated or not and also decide whether the real-time plots should be shown.
* Contributors: Giuseppe Silano, Luigi Iannelli

0.1.6 (2018-06-18)
------------------
* A plot relating to the attitude of the aircraft has been included in the launch files of the task1 and task2 as described in the 26th MED18 Aerial Robotics and Perception Challenge.
* The hovering and waypoints publishers have been inserted as nodes in the repository.
* Task2 has been added. Such task models the drone trajectory tracking (it is modeled by using a sequence of waypoints). The same task was integrated also with the Parrot Bebop2 control algorithm interface.
* A waypoint filter has been added to avoid system instabilities.
* Bug fixing. In particular, it has been fixed the bag into the Extended Kalman Filter: the linear velocity in ABC reference system was used to control the UAV.
* Typos fixing
* Contributors: Giuseppe Silano, Pasquale Opppido, Luigi Iannelli

0.1.5 (2018-06-13)
------------------
* The controller node has been developed that can receive data from and send commands to the Parrot Bebop 2. This node is integrated with the status estimator, i.e., the Extended Kalman filter and the position control algorithm. The filters only take into account the noisy position and the linear speed.
*The wind gusts have been integrated into the Gazebo 3D environment according to the task1 of the 26th MED aerial robotics and perception challenge.
* Contributors: Pasquale Oppido, Giuseppe Silano, Luigi Iannelli

0.1.4 (2018-06-11)
------------------
* Added the Kalman filter
* Added plots in the launch file to monitor the position and linear velocity errors between the Kalman filter output and the odometry ground truth values. In order to develop such functionality, the teamsannio messages have been created.
* A yaml file needed to set up the Kalman filter has been created. Such file contains the tuning matrix of the filter and the standard deviations that characterize the odometry virtual sensor.
* The Kalman filter works with the noisy attitude (standard deviation 0.017). This function considers the nonideality of the attitude filter onboard the quadrotor.
* Contributors: Pasquale Oppido, Giuseppe Silano, Luigi Iannelli

0.1.3 (2018-05-25)
-----------
* Added the data storage section in the position controller file. This section allows to storage, in suitable csv files, the aircraft and controller states.
* Bug fixing
* The sim time has been added in the launch file. Now the position controller node uses the simulation and not wall clock time.
* Contributors: Giuseppe Silano, Luigi Iannelli

0.1.2 (2018-05-15)
------------------
* Added the basic.world file
* The launch files have been modified considering the ROS package architecture
* The iteration number has been moved from 1000 to 50 in order to speed up the simulation
* Contributors: Giuseppe Silano, Luigi Iannelli

0.1.1 (2018-05-13)
------------------
* Fixed issues in the control law development
* Contributors: Giuseppe Silano, Pasquale Oppido, Luigi Iannelli

0.1.0 (2018-04-30)
------------------
* Initial Ubuntu package release
* Contributors: Giuseppe Silano, Pasquale Oppido, Luigi Iannelli
