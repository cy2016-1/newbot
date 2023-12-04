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

#报错:libcblas.so.3: cannot open shared object file: No such file or directory
#解决方法：
sudo apt-get install libatlas-base-dev

#如果唤醒无反应
#解决方法：
#注意：如果USB麦克风声音小，要在alsamixer里调到最大
#先按F6选择USB麦克风，然后调整Capture的MIC到100

sudo apt install pulseaudio
pulseaudio --start
或者
systemctl --user enable pulseaudio
systemctl --user start pulseaudio

#这两个命令查看默认设备，前面有星号的代表默认：
pacmd list-sinks | grep -e 'index:' -e 'name:'
pacmd list-sources | grep -e 'index:' -e 'name:'

#如果默认的设备不对:
#设置默认扬声器：
pacmd set-default-sink n
#设置默认麦克风：
pacmd set-default-source n
#把这里的n替换成实际的值
```



###### sonowboy训练链接：

https://snowboy.hahack.com/
