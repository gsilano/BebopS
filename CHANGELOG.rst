^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teamsannio_med_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.6 (2018-06-14)
------------------
* A plot relating to the attitude of the aircraft has been included in the launch files of the task1 and task2 as descripted in the 26th MED18 Aerial Robotics and Perception Challenge.
* The hovering and waypoints publishers have been inserted as nodes in the repository.
* Task2 has been added. Such task models the drone the trajectory tracking (it is modeled by using a sequence of waypoints). The same task was integrated also with the Parrot Bebop2 control algorithm interface.
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
* added the basic.world file inside the repository
* the launch files have beed modified ad hoc
* the ieration numbers has been moved from 1000 to 50 in order to speed up the simulation
* Contributors: Giuseppe Silano, Luigi Iannelli

0.1.1 (2018-05-13)
------------------
* fixed issues in the control law development
* Contributors: Giuseppe Silano, Pasquale Oppido, Luigi Iannelli

0.1.0 (2018-04-30)
------------------
* initial Ubuntu package release
* Contributors: Giuseppe Silano, Pasquale Oppido, Luigi Iannelli

