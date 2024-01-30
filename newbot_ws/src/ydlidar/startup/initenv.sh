#!/bin/bash
#echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar.rules

#echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-V2.rules

#echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-2303.rules

#Bus 004 Device 002: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter
echo  'KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' >/etc/udev/rules.d/ydlidar-ch341.rules

service udev reload
sleep 2
service udev restart

