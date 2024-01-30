###### 开机远程登陆和挂载nfs：

```
ssh orangepi@orangepi.local
sudo mount -t nfs -o nolock 192.168.0.109:/home/luowei/workspace/nfs ~/nfs
```



###### 安装snowboy依赖库和常见问题：

```shell
#注意：使用apt安装pyaudio
sudo apt update
sudo apt install python3-pyaudio

#此时运行代码会报错
cd ~/nfs/newbot/python_demo/circular_lcd
python3 wakeup.py
#报错:libcblas.so.3: cannot open shared object file: No such file or directory
#解决方法：
sudo apt-get install libatlas-base-dev

#运行代码如果唤醒无反应
python3 wakeup.py
#解决方法：
alsamixer
#如果USB麦克风声音小，要在alsamixer里调到最大
#具体操作:先按F6选择USB麦克风设备,然后按F5,然后调整CAPTURE的Mic到100


sudo apt install pulseaudio
pulseaudio --start
或者
systemctl --user enable pulseaudio
systemctl --user start pulseaudio

#这两个命令查看默认设备，前面有星号的代表默认：
pacmd list-sinks | grep -e 'index:' -e 'name:'
pacmd list-sources | grep -e 'index:' -e 'name:'
#如果列表里根本没有USB设备，则重新安装pulseaudio
sudo apt remove pulseaudio
sudo apt install pulseaudio

#如果默认的设备麦克风设备不是usb
#设置默认扬声器：
pacmd set-default-sink n
#设置默认麦克风为usb设备：
pacmd set-default-source n
#把这里的n替换成实际的值

#重新运行代码测试唤醒
python3 wakeup.py
```



###### sonowboy国内训练链接：

https://snowboy.hahack.com/

