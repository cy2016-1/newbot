###### 开机远程登陆和挂载nfs：

```
ssh orangepi@orangepi.local
sudo mount -t nfs -o nolock 192.168.0.109:/home/luowei/workspace/nfs ~/nfs
```



###### 安装依赖库：

```shell
#安装pip
sudo apt install python3-pip
pip config set global.index-url https://mirrors.aliyun.com/pypi/simple/
pip install -U pip

#安装GPIO、SPI、UART包
pip install gpio
pip install python-periphery
pip3 install pyserial

#注意：使用apt安装opencv
sudo apt update
sudo apt install python3-opencv

#配置spi使能
sudo orangepi-config
#在弹出的界面中打开spi和串口选项,具体参考官方用户手册，然后重启
```



###### 运行代码：

```shell
cd ~/nfs/newbot/python_demo/circular_lcd

#lcd显示测试
python3 lcd.py

#uart控制底盘测试
python3 uart.py

#表情包显示测试
cp image.zip ~
cd ~ && unzip image.zip
cd -
python3 emoji.py

#唤醒显示表情包测试
#首先参考另一个说明文件安装snowboy环境
python3 wakeup.py
```

