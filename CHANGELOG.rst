^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package BebopS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2019-XX-XX)
------------------
* The interface between the position_controller and the Parrot Sphinx simulator has been made
* Added the hovering example file to publish the waypoint when the Parrot-Sphinx simulator is in the loop
* Added the waypoint example file to publish waypoints when the Parrot-Sphinx simulator is in the loop
* The task1_world_with_Sphinx and task2_world_with_Sphinx launch files allow to achieve hovering and trajectory tracking by using the Parrot-Sphinx simulator.
* Added the awk script for catching data from the Sphinx data logger
* Fixed bug in the waypoint filter initialization
* Added a custom message "Sphinx_msgs.msg" to use the data come from the awk script into the ROS net. The topics has been also integrated in the position_control ROS node with Sphinx
* Added a timer before starting the controller. It allows to Parrot Bebop to reach the hovering position, one meter from the ground.
* Contributors: Giuseppe Silano, Peter Griggs

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
* A plot relating to the attitude of the aircraft has been included in the launch files of the task1 and task2 as descripted in the 26th MED18 Aerial Robotics and Perception Challenge.
* The hovering and waypoints publishers have been inserted as nodes in the repository.
* Task2 has been added. Such task models the drone the trajectory tracking (it is modeled by using a sequence of waypoints). The same task was integrated also with the Parrot Bebop2 control algorithm interface.
* A waypoint filter has been added to avoid system instabilities.
* Bug fixing. In particular it has been fixed the bag into the Extended Kalman Filter: the linear velocity in ABC reference system was used to control the UAV.
* Typos fixing
* Contributors: Giuseppe Silano, Pasquale Opppido, Luigi Iannelli

0.1.5 (2018-06-13)
------------------
* The controller node able to reiceive data from and to send commands to the Parrot Bebop2 has been developed. Such node is integrated with the state estimator, i.e., the Extended Kalman filter, and the position control algorithm. The filters takes into account only the noisy position and linear velocity.
* The wind gusts have been integrated into the Gazebo 3D environment according to the task1 of the 26th MED aerial robotics and perception challenge. 
* Contributors: Pasquale Oppido, Giuseppe Silano, Luigi Iannelli

0.1.4 (2018-06-11)
------------------
* added the Kalman filter
* added plots in the launch file to monitor the position and linear velocity errors between the Kalman filter output and the odometry ground truth values. In order to develop such functionality, the teamsannio messages have been created.
* A yaml file needed to set up the Kalman filter has been created. Such file contains the tuning matrix of the filter and the standard deviations that characterize the odometry virtual sensor.
* The Kalman filter works with the noisy attitude (standard deviation 0.017). This function considers the nonideality of the attitude filter onboard the quadrotor.   
* Contributors: Pasquale Oppido, Giuseppe Silano, Luigi Iannelli

0.1.3 (2018-05-25)
-----------
* added the data storage section. Such section has been inserted in the position controller node and allows to storage, in defined csv files, the aircraft and controller state.
* bug fixing
* the sim time has been added in the launch file. Now the position controller node uses the simulation and not wall clock time.
* Contributors: Giuseppe Silano, Luigi Iannelli

0.1.2 (2018-05-15)
------------------
* added the basic.world file
* the launch files have been modified considering the ROS package architecture
* the iteration number has been moved from 1000 to 50 in order to speed up the simulation
* Contributors: Giuseppe Silano, Luigi Iannelli

0.1.1 (2018-05-13)
------------------
* fixed issues in the control law development
* Contributors: Giuseppe Silano, Pasquale Oppido, Luigi Iannelli

0.1.0 (2018-04-30)
------------------
* initial Ubuntu package release
* Contributors: Giuseppe Silano, Pasquale Oppido, Luigi Iannelli

