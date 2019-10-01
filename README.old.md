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

##  reading sensor data and saving as xls file from a rosbag (Laptop)
Downlowd "bamboobota_2018-10-02-10-43-35_0.bag" from https://drive.google.com/drive/u/1/folders/1BE6V8jH0n0DXCsdMW8F54HbpBhaxlbCC
```
laptop$ byobu
```
terminal 1:
```
laptop$ roscore
```
terminal 2:
```
laptop$ roslaunch robotx_bionic data_to_xls.launch starting_time:=2018-10-02-10-43-00
```
terminal 3:
```
laptop$  rosbag play BAGNAME.bag
```

## Recording Temparature
Connect to "jason_car" Wi-Fi router

```
laptop$ ssh agvduck@10.42.0.1 pwd: agvduck
```
```
rpi3$ byobu
```
terminal 1: Turn on current sensor
```
rpi3$ source ~/robotx_bionic/environment.sh
rpi3$ roslaunch rasberrypi_sensors temp_current.launch 
```
terminal 2: Turn on GPS
```
rpi3$ source ~/robotx_bionic/environment.sh
rpi3$ roslaunch robotx_bionic sensor.launch 
```
terminal 3:
```
rpi3$ source ~/robotx_bionic/environment.sh
rpi3$ rosbag record /fix /temp_current_8A_node/temp_current --chunksize=1
```

## Fix active bag

```
laptop$ rosbag reindex xxx.bag.active
laptop$ rosbag fix xxx.bag.active xxx.bag
laptop$ rm -f *.orig.active
```