# robotx_bionic
The repo for bionic technology project

# how to run 
##  bamboobot (NVidia TX2)
Velodyne
```
$ cd ~/robotx_nctu
$ source environment.sh
$ roslaunch velodyne_pointcloud VLP16_points.launch veh:=bamboobota_velodyne frame_id:=bamboobota_velodyne
```

ZED Camera
```
$ cd ~/zed
$ source catkin_ws/devel/setup.bash
$ roslaunch zed_wrapper zed_camera.launch veh:=bamboobota
```

Apriltag Localization 
```
$ cd ~/robotx_bionic
$ source environment.sh
$ roslaunch robotx_bionic apriltag_localization.launch veh:=bamboobota camera_name:=rgb
```

Apriltag Localization in Gazebo 
```
lauanch gazebo world
$ cd ~/robotx_gazebo
$ source environment.sh
$ roslaunch robotx_gazebo bamboolake_bionic.launch 

run waypoint navigation contoller
$ cd ~/robotx_gazebo
$ source environment.sh
$ rosrun robotx_gazebo WAMV_PID_controller.py

gps imu localization
$ cd ~/robotx_nctu
$ source environment.sh
$ roslaunch localization gps_imu_localization_gazebo.launch

visualization
$ cd ~/robotx_gazebo
$ source environment.sh
$ rviz

set waypoints
$ cd ~/robotx_gazebo
$ source environment.sh
$ rosservice call /add_waypoint  10  0 0 0 && rosservice call /add_waypoint  15  5 0 0 && rosservice call /add_waypoint  15 10 0 0 && rosservice call /add_waypoint  20 15 0 0 && rosservice call /add_waypoint  25 15 0 0 && rosservice call /add_waypoint  30 10 0 0 && rosservice call /add_waypoint  30  5 0 0 && rosservice call /add_waypoint  35  0 0 0 && rosservice call /add_waypoint  40  0 0 0
start navigation
$ rosservice call /start_waypoint_nav "{}"

apriltag localization
$ roslaunch robotx_bionic wamv_localization_path.launch veh:=wamv

draw path
$ rostopic pub /wamv_path/channel std_msgs/Int32 "data: 1" 

```

##  environment sensors (Pi 3)
Temparature and Pressure Sensors
```
$ cd ~/robotx_bionic
$ source environment.sh
$ roslaunch robotx_bionic sensor_measurement.launch veh:=bamboobota
```

##  analysis (Laptop)

## recording temparature and pressure 
Connect to "sean-rv" Wi-Fi router
```
laptop$ ssh sean_devel@10.42.0.1 pwd: sean85914
rpi3$ byobu
```
terminal 1:
```
rpi3$ source ~/robotx_bionic/environment.sh
rpi3$ $export ROS_MASTER_URI=http://10.42.0.1:11311/
rpi3$ roslaunch robotx_bionic sensor_measurement.launch veh:=bamboobota
```
terminal 2:
```
rpi3$ source ~/robotx_bionic/environment.sh
rpi3$ export ROS_MASTER_URI=http://10.42.0.1:11311/
rpi3$ rosbag record -a -o bamboobota_tem_press --split 1024
```

