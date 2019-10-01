sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"2303\", ATTRS{manufacturer}==\"Prolific Technology Inc. \",MODE=\"0777\",SYMLINK+=\"sensor_gps\"">/etc/udev/rules.d/50-duckiepond_usb_port.rules'
sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"9d0f\", ATTRS{manufacturer}==\"SparkFun\", MODE=\"0777\", SYMLINK+=\"sensor_imu\"" >> /etc/udev/rules.d/50-duckiepond_usb_port.rules'
sudo udevadm trigger

