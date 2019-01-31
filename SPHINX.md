# SPHINX Package for Bebop

## Running code in Sphinx
1. Start firmware
```bash
$ sudo systemctl start firmwared.service
$ sudo firmwared
```
* Check if firmware is running properly
```bash
$ fdc ping
PONG
```

2. Start Sphinx
```bash
$ sphinx --datalog /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone::with_front_cam=false
```

3. Cd into the packages launch files folder
```bash
$ cd launch/
```

4. Run the launch file with ROS, will start the Bebop Autonomy package as well as this package
```bash
$ roslaunch sphinx_controller_test.launch
```

*Note*: make sure that the Bebop Autonomy launch file at bebop_node.launch has the ip address 10.202.0.1

## Changes made to original package
Since the Bebop already implements the inner loop controller, we have stripped out the logic for the functions related to the inner loop, like the AttitudeController. Instead we just get the waypoints and trajectory, and use the Position Controller to generate the control signals and use the Odometry data as feedback. In the OdometryCallback function in position_controller_node.cpp, we check if the drone has not taken off, since this must happen for the drone to fly in Sphinx. All of the flight is controlled by two functions: SendTakeoffMsg() and SendPilotMsg(). In SendPilotMsg(), we use the roll angle, pitch angle, and vertical velocity to compute a linear vector of control signals.