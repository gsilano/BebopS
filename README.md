
# Hover Sphinx Package Docs


## How To Run in Sphinx

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
$ roslaunch Takeoff.launch
```

## Structure of Files & Code

The main code for this package is located in the `src/takeoff_sphinx_test_node.cpp` file. The `main()` function is the entry point for this package. Currently, it sends a message to the rostopic `/bebop/takeoff`, which makes the drone takeoff and then it sends a message to the rostopic `/bebop/land`, which makes the drone land. The `odomCallback()` function simply processes an odometry message and prints out the `(x,y,z)` coordinates of the time from the `/bebop/odom` rostopic.


## Known bugs

Currently, the odometry readings are not being printed when we are able to successfully takeoff and land. When we subscribe to the `/bebop/odom` topic and then call `spin()`, we can get the odometry readings, but this blocks the call to `/bebop/land` so that the drone cannot land. This is because the `main()` function only uses a single thread, and `spin()` repeatedly calls any callbacks in the callback queue.