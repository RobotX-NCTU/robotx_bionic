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
$ export ROS_MASTER_URI=http://192.168.2.102:11311/
$ cd ~/robotx_bionic
$ source environment.sh
$ roslaunch robotx_bionic sensor_measurement.launch veh:=bamboobota
```

##  analysis (Laptop)