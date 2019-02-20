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

##  environment sensors (Pi 3)
Temparature and Pressure Sensors
```
$ cd ~/robotx_bionic
$ source environment.sh
$ roslaunch robotx_bionic sensor_measurement.launch veh:=bamboobota
```

##  analysis (Laptop)
To do

## recording temparature and pressure 
Connect to "sean-rv" Wi-Fi router
```
$ ssh sean_devel@10.42.0.1 pwd: sean85914
$ byobu
```
terminal 1:
```
$ source ~/robotx_bionic/environment.sh
$ $export ROS_MASTER_URI=http://10.42.0.1:11311/
$ roslaunch robotx_bionic sensor_measurement.launch veh:=bamboobota
```
terminal 2:
```
$ source ~/robotx_bionic/environment.sh
$ export ROS_MASTER_URI=http://10.42.0.1:11311/
$ rosbag record -a -o bamboobota_tem_press --split 1024
```
