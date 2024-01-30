#!/usr/bin/env python3
#coding=utf-8
import sys
sys.path.append("/home/orangepi/.local/lib/python3.8/site-packages")
import os
os.close(sys.stderr.fileno()) #关闭声卡的大量警告打印

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import numpy as np
import requests

import random
import time
import queue
import psutil
import cv2
import pulsectl

import asr
import llm
import record
import tts
import mail
import qr_code



def play_music(file_name):
    # pygame.mixer.init()
    # pygame.mixer.music.set_volume(1.0) #音量0~1
    # pygame.mixer.music.load(file_name)
    # pygame.mixer.music.play()
    #
    # while pygame.mixer.music.get_busy():
    #     time.sleep(0.1)
    #
    # pygame.mixer.music.stop()

    cmd = "play \"%s\" >/dev/null 2>&1"%(file_name)
    os.system(cmd)


def chat_with_llm(audio_file_name, asr_client, llm_client):
    if audio_file_name==None:
        print("没有检测到语音")
        answer = "我似乎什么也没有听到"
    else:
        print('录音完成')
        wav_data = asr.get_file_content(audio_file_name)
        text = asr_client.audio_to_text(wav_data)
        print("User: %s"%text)
        if text=="":
            print("没有从语音中识别到文字")
            answer = "我好像没有听清你在说什么"
        else:
            text_input = "你是我的宠物机器人，你的名字叫小白，你要用一只软萌机器人的语气和我聊天，回答字数必须小于50字，现在我聊天的内容是：" + text
            answer = llm_client.chat(text_input)
            print("AI: %s"%answer)
            if answer == "":
                print("大模型聊天回复为空")  # 可能问了不合法的问题，不合法的问题有可能返回的是空
                answer = "这个问题我还不知道呢"
    return answer


def check_music_playing():
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] == 'play':
            return True
    return False

def kill_music_process():
    for proc in psutil.process_iter(['name', 'cmdline']):
        if proc.info['name'] == 'play' and 'play' in proc.info['cmdline']:
            proc.kill()




def decode_encode_jpeg(jpeg_data, calibration_file):
    # 读取JPEG数据
    image = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), -1)
    
    # 创建FileStorage对象
    fs = cv2.FileStorage(calibration_file, cv2.FILE_STORAGE_READ)
    
    # 读取相机内参和畸变系数
    camera_matrix = fs.getNode("CameraMat").mat()
    dist_coeffs = fs.getNode("DistCoeff").mat()
    
    # 关闭FileStorage对象
    fs.release()
    
    h,w,c = image.shape
    # 鱼眼矫正
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, dist_coeffs, np.eye(3), camera_matrix, (w,h), cv2.CV_16SC2)
    undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)

    undistorted_image = undistorted_image[0:0+720, 160:160+960]

    # 将OpenCV图像编码为JPEG数据
    success, encoded_image = cv2.imencode('.jpg', undistorted_image, [cv2.IMWRITE_JPEG_QUALITY, 100])
    undistorted_jpeg_data = encoded_image.tobytes()
    
    return undistorted_jpeg_data


def tts_play(tts_str,tts_client,tts_local_client,enable_wakeup_pub,turn_off_wakeup=True):

    mp3_data = tts_client.text_to_speech(tts_str)

    if turn_off_wakeup:
        msg = Bool()
        msg.data = False  # 获取到语音命令，自己开始发声音，关闭唤醒
        enable_wakeup_pub.publish(msg)

    if mp3_data != None:
        with open("tts.mp3", "wb") as file:
            file.write(mp3_data)
        play_music("tts.mp3")
    else:  # edge tts调用失败，则用本地TTS播放
        tts_local_client.text_to_speech(tts_str)

    if turn_off_wakeup:
        msg = Bool()
        msg.data = True  # 完成语音命令，打开唤醒
        enable_wakeup_pub.publish(msg)

def get_current_volume():
    # 创建 PulseAudio 的客户端对象
    pulse = pulsectl.Pulse('volume-info')

    # 获取默认输入设备的音量信息
    sink_info = pulse.get_sink_by_name('@DEFAULT_SINK@')
    volume = round(sink_info.volume.value_flat * 100)

    # 关闭 PulseAudio 客户端连接
    pulse.close()

    return volume


class Audio:
    def __init__(self):
        self.tts_str_queue = queue.Queue(2)
        self.new_tts_flag = False
        self.image_sub = None
        self.frame_cnt = 0

    def do_dance(self, rate, cmd_vel_pub, check_music):
        speed = Twist()

        zxts = [(-1.0, 0, 0.5), (0, 0, 0.1),
                (1.0, 0, 0.5), (0, 0, 0.1),

                (0, 0.04, 0.5), (0, 0, 0.1),
                (0, -0.04, 0.5), (0, 0, 0.1),

                (1.0, 0, 0.5), (0, 0, 0.1),
                (-1.0, 0, 0.5), (0, 0, 0.1),

                (0, -0.04, 0.5), (0, 0, 0.1),
                (0, 0.04, 0.5), (0, 0, 0.1),
                ]

        while (not self.new_tts_flag and not rospy.is_shutdown()):  # 如果self.new_tts_flag==0则一直循环
            for z, x, t in zxts:
                speed.angular.z = z
                speed.linear.x = x
                cmd_vel_pub.publish(speed)

                for i in range(int(t / 0.1)):  # 0.5/0.1=5
                    rate.sleep()  # 0.1s

                if self.new_tts_flag:  # 如果收到新的命令，则退出目标
                    break

            if check_music == True:
                if check_music_playing() == False:
                    print("音乐播放已结束，停止舞蹈")
                    break

        # 停止
        speed.angular.z = 0
        speed.linear.x = 0
        cmd_vel_pub.publish(speed)

        # 关闭播放音乐
        os.system("killall play >/dev/null 2>&1")


    def scan_qr_code(self):
        self.frame_cnt = 0
        if self.image_sub != None:
            self.image_sub.unregister()  # 取消订阅
        # 订阅消息
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.raw_image_callback)

    def take_a_photo(self):
        if self.image_sub != None:
            self.image_sub.unregister()  # 取消订阅
        # 订阅消息
        self.image_sub = rospy.Subscriber('/image_raw/compressed', CompressedImage, self.compressed_image_callback)

    def raw_image_callback(self,msg):

        self.frame_cnt += 1

        # 将ROS中的Image消息转换为numpy数组
        img_array = np.frombuffer(msg.data, np.uint8)
        image = np.reshape(img_array, (msg.height, msg.width, -1))
        # 解析二维码
        try:
            text = qr_code.scan_qrcode(image)
        except Exception as e:  # 没有二维码的情况会进入异常
            text = ""

        if text != "" or self.frame_cnt > 1000:  # 0.033s * 1000 = 33s
            self.image_sub.unregister()  # 取消订阅
            self.frame_cnt = 0
            msg = String()
            if text == "":
                text = "未能识别到二维码"
            else:
                current_dir = os.path.dirname(os.path.realpath(__file__))
                deng_file_name = os.path.join(current_dir, "sound", "cut_deng.wav")
                play_music(deng_file_name)
                text = "识别到二维码内容:" + text
            msg.data = text
            self.sub_tts_callback(msg)


    def sub_tts_callback(self,msg):
        #print("audio sub_tts_callback:", msg.data)

        split_str = msg.data.split("@")
        if split_str[0] == "exit_wake_up":  # 退出唤醒不用打断之前的任务
            current_dir = os.path.dirname(os.path.realpath(__file__))
            bye_file_name = os.path.join(current_dir, "sound", "bye.mp3")
            play_music(bye_file_name)
            return

        # 已经重新唤醒，关闭正在播放的内容
        os.system("killall play >/dev/null 2>&1")
        self.tts_str_queue.put(msg.data)
        self.new_tts_flag = True


    def compressed_image_callback(self,msg):
        
        self.image_sub.unregister()  # 取消订阅

        # with open("photo.jpg", 'wb') as f:
        #     f.write(msg.data)

        current_dir = os.path.dirname(os.path.realpath(__file__))
        calibration_file = os.path.join(current_dir, "fisheye.yml")
        undistorted_jpeg_data = decode_encode_jpeg(msg.data, calibration_file)

        mail.send_mail(undistorted_jpeg_data)
        print("拍照成功")


    def main_loop(self):
        rospy.init_node("tts")

        asr_client = asr.create_asr("baidu")
        llm_client = llm.create_llm("xunfei")
        tts_client = tts.create_tts("edge")
        tts_local_client = tts.create_tts("local")

        current_dir = os.path.dirname(os.path.realpath(__file__))
        started_file_name = os.path.join(current_dir, "sound", "start.wav")
        detected_file_name = os.path.join(current_dir, "sound", "cut_deng.wav")
        finished_file_name = os.path.join(current_dir, "sound", "dong.wav")
        files = os.listdir(os.path.join(current_dir, "music"))
        music_file_names = []
        for file in files:
            music_file_names.append(os.path.join(current_dir, "music", file))

        tts_sub = rospy.Subscriber("/tts", String, self.sub_tts_callback, queue_size=2)

        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        enable_wakeup_pub = rospy.Publisher('/enable_wakeup', Bool, queue_size=10)

        play_music(started_file_name)
        tts_str = "主人您好，我是你的小白机器人"
        tts_play(tts_str, tts_client, tts_local_client, enable_wakeup_pub, turn_off_wakeup=True)

        rate = rospy.Rate(10)  # 10Hz 0.1s

        while not rospy.is_shutdown():
            try:
                tts_string = self.tts_str_queue.get(timeout=1)  # 会阻塞  #等待1秒，如果还是不能取到数据，则抛出异常。
                self.new_tts_flag = False
            except:
                rate.sleep()  # 0.1s
                continue

            print("TTS:", tts_string)

            tts_str_list = tts_string.split("#")
            if len(tts_str_list) == 1:
                tts_str = tts_str_list[0]
                key_str = None
            else:
                tts_str = tts_str_list[0]
                key_str = tts_str_list[1]

            if key_str == "voice_up":
                os.system("amixer -D pulse set Master 20%+")
                tts_str += "到百分之%d" % (get_current_volume())
            elif key_str == "voice_down":
                os.system("amixer -D pulse set Master 20%-")
                tts_str += "到百分之%d" % (get_current_volume())
            elif key_str == "chatgpt":
                tts_str = ""

            if tts_str != "":  # 回复命令，可能回复词会和命令词一样，所以要关闭唤醒，不可打断
                tts_play(tts_str, tts_client, tts_local_client, enable_wakeup_pub, turn_off_wakeup=True)

            if key_str == "photo":
                self.take_a_photo()
            elif key_str == "scan":
                self.scan_qr_code()
            elif key_str == "sing":  # 唱歌模式
                play_music(random.choice(music_file_names))
            elif key_str == "dance":  # 跳舞模式
                self.do_dance(rate, cmd_vel_pub, check_music=False)
            elif key_str == "sing_and_dance":  # 唱跳模式
                cmd = "play \"%s\" >/dev/null 2>&1 &" % (random.choice(music_file_names))  # 后台播放
                os.system(cmd)
                self.do_dance(rate, cmd_vel_pub, check_music=True)
            elif key_str == "chatgpt":  # 聊天模式
                play_music(detected_file_name)

                audio_file_name = record.record_audio()
                if self.new_tts_flag:
                    continue

                play_music(finished_file_name)

                ai_answer = chat_with_llm(audio_file_name, asr_client, llm_client)
                if self.new_tts_flag:
                    continue

                tts_play(ai_answer, tts_client, tts_local_client, enable_wakeup_pub, turn_off_wakeup=True)

            rate.sleep()  # 0.1s


if __name__ == "__main__":
    audio = Audio()
    audio.main_loop()