## Extended_Kalman_Filter [CarND]

In this project, we have to create Extended Kalman Filter using C++ and use it to estimate the state of moved object of interest with RADAR measurements and noisy LiDAR measurements.

This project involves [Udacity Self-Driving Car Engineer Nanodegree Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)

##Prerequisites
####The project has the following dependencies:
* cmake = 3.16.3
* make = 4.2.1
* g++ = 9.3.0
* [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) and [implementation method](https://github.com/udacity/CarND-Path-Planning-Project/blob/master/install-ubuntu.sh)

##Build and Run
Before Start this project, you should Create "build" folder and right this comments in terminal.

```bash
$> mkdir build
$> cd build
$> cmake ..
$> make
$> ./ExtendedKF
```
After that, Start the Term 2 Simulator
```bash
$> cd term2_sim_linux
$> ./term2_sim.x86_64
```
##Demo
###Dataset [1]
[*zoom in*]
![ZOOM_IN](result_data/Dataset1_zoom_in.gif)
[*zoom out*]
![ZOOM_IN](result_data/Dataset1_zoom_out.gif)
###Dataset [2]
[*zoom in*]
![ZOOM_IN](result_data/Dataset2_zoom_in.gif)
[*zoom out*]
![ZOOM_IN](result_data/Dataset2_zoom_out.gif)

###Protocol
The following are the default protocols that main.cpp uses for uWebSocketIO: This uWebSocketIO is used to communicate with the simulator.

[**INPUT**]
The value that the Simulator provides to the C++ program:
```
["sensor_measurement"] => The measurement that the simulator observed [lidar/radar]
```
[**OUTPUT**]
The value that the C++ program provides to the Simulator:
```
["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]
```

#### Notice
For comprehensive instructions on how to install and run project, please, refer to the following repo, which was used as a skeleton for this project: https://github.com/udacity/CarND-Extended-Kalman-Filter-Project.
