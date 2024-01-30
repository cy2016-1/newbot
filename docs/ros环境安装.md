###### 开机远程登陆和挂载nfs：

```shell
ssh orangepi@orangepi.local

sudo mount -t nfs -o nolock 192.168.0.109:/home/luowei/workspace/nfs ~/nfs
orangepi
```

###### ubuntu20.04安装ROS：

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update
#安装ros完整版，等待数分钟完成
sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

###### 配置多机通信：

```shell
vim ~/.bashrc
host_ip=$(hostname -I | awk '{print $1}')
if [ -z $host_ip ]; then
  host_ip="192.168.12.1" #香橙派AP模式IP
fi
export ROS_IP=$host_ip
export ROS_HOSTNAME=$host_ip
export ROS_MASTER_URI=http://$host_ip:11311
:wq
source ~/.bashrc
```

###### 安装常用的ROS包：

```shell
#安装键盘遥控包
sudo apt install ros-noetic-teleop-twist-keyboard

#安装导航规划相关的包
sudo apt install ros-noetic-move-base-msgs
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-base-local-planner
sudo apt install ros-noetic-dwa-local-planner
sudo apt install ros-noetic-teb-local-planner
sudo apt install ros-noetic-global-planner

sudo apt install ros-noetic-gmapping
sudo apt install ros-noetic-amcl

#安装EAI YDLidar库
cd ~/nfs/newbot/newbot_ws
cd src/ydlidar/YDLidar-SDK
rm -r build
mkdir build && cd build
cmake ..
make
sudo make install
```

###### 使用ROS新建python项目：

```shell
cd ~/nfs/newbot/newbot_ws/src
catkin_create_pkg <package_name> rospy

#创建文件scripts/xxx.py:
#!/usr/bin/env python3
#coding=utf-8
...

sudo chmod +x src/package_name/scripts/xxx.py
```

###### 安装常用的python包：

```shell
pip3 install edge_tts
pip install pygame
pip3 install pyttsx3

pip install SpeechRecognition
sudo apt install python3-websocket

pip install pyzbar
pip install opencv-python
pip install pulsectl
```

###### 常见问题及解决方法：

```shell
#扬声器在数据传输或者运行程序时杂音很大
#解决方法：加粗电源线！！！加粗电源线！！！

#报错：
play test.mp3
#play WARN alsa: can't encode 0-bit Unknown or not applicable
#play FAIL formats: no handler for file extension `mp3'
#解决方法：
export AUDIODRIVER=alsa

#报错：
play test.mp3
#play FAIL formats: no handler for file extension `mp3'
#解决方法：
sudo apt-get install libsox-fmt-mp3


#解决开启AP之后nmcli dev中wlan0显示unmanaged
#编写脚本wifi.sh：
echo "orangepi" | sudo -S create_ap --fix-unmanaged #这句作用相当于关闭AP
sleep 5 #等待进入station模式
nmcli device wifi connect "TP-LINK_E164" password "" #连接家庭WIFI
echo "orangepi" | sudo -S reboot #重启以让ROS IP生效
#后台运行脚本：
bash wifi.sh &
```

