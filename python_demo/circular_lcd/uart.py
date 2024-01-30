import serial #pip install pyserial
import struct
import time
import threading

'''
和底盘的通信协议：
typedef struct
{
    unsigned char head1;//数据头1 'S' 0x53
    unsigned char head2;//数据头2 'T' 0x54
    unsigned char struct_size;//结构体长度

    short pwm1;//油门PWM1
    short pwm2;//油门PWM2

    unsigned char end1;//数据尾1 'U' 0x55
    unsigned char end2;//数据尾2 'V' 0x56
    unsigned char end3;//数据尾3 '\r' 0x0d
    unsigned char end4;//数据尾4 '\n' 0x0a
}CmdData;
'''

class Uart:
    def __init__(self):
        self.ser = serial.Serial(port="/dev/ttyS2", baudrate=115200)
        self.pwm1 = 0
        self.pwm2 = 0
        threading.Timer(0.02, self._send_data).start()  # 每隔20ms触发一次定时器

    def close(self):
        self.ser.close() # 关闭串口

    def set_pwm(self,pwm1,pwm2):
        self.pwm1 = pwm1
        self.pwm2 = pwm2

    def _send_data(self):
        data = struct.pack('<BBBhhBBBBB', 0x53, 0x54, 12,
                           self.pwm1, self.pwm2,1,
                           0x55,0x56,0x0D,0x0A)

        # 发送数据
        self.ser.write(data)

        threading.Timer(0.02, self._send_data).start()  # 每隔20ms触发一次定时器,必须反复调用。。。


if __name__=="__main__":
    uart = Uart()
    while True:
        uart.set_pwm(-2000,2000) #左转
        time.sleep(0.2)

        uart.set_pwm(0,0)
        time.sleep(1)

        uart.set_pwm(2000, -2000) #右转
        time.sleep(0.2)

        uart.set_pwm(0, 0)
        time.sleep(1)

