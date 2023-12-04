

#### 安装和挂载NFS：

###### 电脑作为服务端：

```shell
sudo apt install nfs-kernel-server
sudo gedit /etc/exports
末尾增加一行：/home/luowei/workspace/nfs *(rw,sync,no_root_squash)
showmount -e localhost
```



###### 板子作为客户端：

```shell
ssh orangepi@板子ip地址
sudo apt install nfs-common
mkdir ~/nfs
sudo mount -t nfs -o nolock 192.168.0.109:/home/luowei/workspace/nfs ~/nfs
```

 

#### SSH如何通过用户名登陆板子：

###### 安装和启用Avahi服务：

```shell
sudo apt-get install avahi-daemon
sudo vim /etc/avahi/avahi-daemon.conf
#找到#host-name=foo这一行，取消注释并将其修改为：
host-name=orangepi
#输入:wq保存并关闭

#开启服务
sudo service avahi-daemon restart
```



###### 测试能否使用用户名登陆：

```shell
#可以先测试能否ping通
ping orangepi.local

#使用用户名登陆
ssh orangepi@orangepi.local
```

