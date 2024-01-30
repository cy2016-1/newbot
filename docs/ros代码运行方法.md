#### 手动执行程序：

```shell
#从百度云控制台和科大迅飞控制台申请语音识别和迅飞大模型的API KEY
#手动执行如下export命令，或者写入~/.bashrc中自动执行export
export XUNFEI_APPID="xxx"
export XUNFEI_APIKEY="xxx"
export XUNFEI_APISECRET="xxx"

export BAIDU_APPID="xxx"
export BAIDU_APIKEY="xxx"
export BADIU_APISECRET="xxx"

cd ~/nfs/newbot/newbot_ws
catkin_make #第一次运行前如果没编译需要编译
source devel/setup.bash
roslaunch pkg_launch all.launch
```

#### 开机自动执行程序：

###### 修改/etc/rc.local内容如下：

```shell
#!/bin/sh -e

sudo chmod 777 /sys/class/gpio/export
sudo chmod 777 /dev/spidev3.0
sudo chmod 777 /sys/class/gpio/export

echo 128 > /sys/class/gpio/export
echo 130 > /sys/class/gpio/export

sudo chmod 777 /sys/class/gpio/gpio128/value
sudo chmod 777 /sys/class/gpio/gpio128/direction
sudo chmod 777 /sys/class/gpio/gpio130/value
sudo chmod 777 /sys/class/gpio/gpio130/direction

gpio mode 20 out && gpio write 20 0

su - orangepi -c "cd /home/orangepi && bash start.sh &"

exit 0
```

###### 新建并修改~/start.sh内容如下：

```shell
sleep 5 #等待网络连接成功

export XUNFEI_APPID="xxx"
export XUNFEI_APIKEY="xxx"
export XUNFEI_APISECRET="xxx"

export BAIDU_APPID="xxx"
export BAIDU_APIKEY="xxx"
export BADIU_APISECRET="xxx"

host_ip=$(hostname -I | awk '{print $1}')
if [ -z $host_ip ]; then
  echo "orangepi" | sudo -S create_ap --no-virt -m nat wlan0 eth0 orangepi orangepi --freq-band 5 &
  host_ip="192.168.12.1"
fi
export ROS_IP=$host_ip
export ROS_HOSTNAME=$host_ip
export ROS_MASTER_URI=http://$host_ip:11311

. /opt/ros/noetic/setup.sh
. ~/newbot_ws/devel/setup.sh
roslaunch pkg_launch all.launch > ~/ros_log
```

