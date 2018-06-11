^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teamsannio_med_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


0.1.4 (2018-06-11)
------------------
* added the Kalman filter
* added plots in the launch file to monitor the position and linear velocity errors between the Kalman filter output and the odometry ground truth values. In order to develop such functionality, the teamsannio messages have been created.
* A yaml file needed to set up the Kalman filter has been created. Such file contains the tuning matrix of the filter and the standard deviations that characterize the odometry virtual sensor.
* The Kalman filter works with the noisy attitude (standard deviation 0.017). This function considers the nonideality of the attitude filter onboard the quadrotor.   
* Contributors: Pasquale Oppido, Giuseppe Silano, Luigi Iannelli

0.1.3 (2018-05-25)
------------------
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

