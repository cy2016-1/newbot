
import rospy
from std_msgs.msg import String

import sys
sys.path.append("/home/orangepi/.local/lib/python3.8/site-packages")
import requests
import os
import pygame
import time
import asyncio
import edge_tts
import pyttsx3
import socket
#import qr_code

import queue

def play_music(file_name):
    pygame.mixer.init()
    pygame.mixer.music.set_volume(1.0) #音量0~1
    pygame.mixer.music.load(file_name)
    pygame.mixer.music.play()

    while pygame.mixer.music.get_busy():
        time.sleep(0.1)

    pygame.mixer.music.stop()
        
   


def tts_baidu(text,output_file_name):

    url='https://fanyi.baidu.com/gettts'
    
    header={
        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/89.0.4389.72 Safari/537.36 Edg/89.0.774.45'
    } #浏览器信息
    
    params={
    'lan': 'zh',#语言类型，zh中文，en英文
    'text': text,
    'spd': 5,#语速,经测试应填写1~6之内的数字
    'source': 'web',
    }
    
    response = requests.get(url,params=params,headers=header)
    
    if response.status_code == 200 and response.content:
        with open(output_file_name,'wb') as f:
            f.write(response.content)
            
        return True
    else:
        return False
        
        


async def tts_edge(text,output_file_name,speaker='Xiaoxiao'):
    #voice="zh-CN-%sNeural"%(speaker)
    voice="Microsoft Server Speech Text to Speech Voice (zh-CN, %sNeural)"%(speaker)
    communicate = edge_tts.Communicate()
    
    # messages,
    # boundary_type=0,
    # codec="audio-24khz-48kbitrate-mono-mp3",
    # voice="Microsoft Server Speech Text to Speech Voice (en-US, AriaNeural)",
    # pitch="+0Hz",
    # rate="+0%",
    # volume="+0%",
    # messages (str or list): A list of SSML strings or a single text.
    # boundery_type (int): The type of boundary to use. 0 for none, 1 for word_boundary, 2 for sentence_boundary.
    # codec (str): The codec to use.
    # voice (str): The voice to use.
    # pitch (str): The pitch to use.
    # rate (str): The rate to use.
    # volume (str): The volume to use.
            
    with open(output_file_name,'wb') as f:
        async for i in communicate.run(text,voice=voice):
            if i[2] is not None:
                f.write(i[2])
                
    return True




def tts_local_say(text):
    engine = pyttsx3.init()
    engine.setProperty('voice', 'zh') #开启支持中文
    engine.say(text)
    engine.runAndWait()
        
        
def tts_run(text,tts_method,speaker="Xiaoxiao"): #Xiaoxiao Xiaoyi Yunxia
    output_file_name = text+'.mp3'

    if os.path.exists(output_file_name): #缓存文件
        play_music(output_file_name)
        #print("%s播放成功" % (speaker))
        return #播放成功则返回

    ret = False
    if tts_method == "edge":
        if 1:
        #try:
            asyncio.get_event_loop().run_until_complete(tts_edge(text,output_file_name,speaker))
            ret = True
        #except:
        else:
            print("edge tts error!")
            ret = False
            tts_method = "baidu"

    if tts_method == "baidu":
        try:
            ret = tts_baidu(text,output_file_name)
            if ret == False:
                print("baidu tts ret False!")
                tts_method = "local"
        except:
            print("baidu tts error!")
            ret = False
            tts_method = "local"


    if ret == True:
        play_music(output_file_name)
        #print("%s播放成功"%(speaker))
        #except:
        #    print("mp3播放失败") #获取mp3成功，但是播放失败了
        #    tts_method = "local"


    if tts_method == "local":
        try:
            tts_local_say(text)
        except:
            print("local tts error!")

def get_host_ip():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(('8.8.8.8', 80))
        ip = sock.getsockname()[0]
        sock.close()
    except:
        sock.close()
        return None

    return ip



        
'''
  
    play_music("boot0.wav")
    
    while True:
        for t in range(5):
            ip_address = get_host_ip()
            print("IP地址:",ip_address)
            
            if ip_address!=None: #10秒内成功获取IP则提前跳出
                break
                
            time.sleep(1)
        
        
        
        if ip_address:
            play_music("boot.wav")
            text = "你好，我是你的机器人助理，已成功连接网络，你可以在浏览器中输入IP地址%s浏览画面"%(ip_address)
            tts_run(text,tts_method="edge")
            exit(0)
            
            
        #IP获取失败
        text = "你好，我是你的机器人助理，网络连接异常，尝试通过摄像头扫描二维码连接无线网"
        tts_run(text,tts_method="edge")
    
    
    
        ret,string = qr_code.get_qr_code(1)
        if ret==False:
            text = "摄像头打开异常"
            tts_run(text,tts_method="edge")
            time.sleep(3)
            continue
        
        
        SSID = None
        PWD = None
        for txt in string.split(";"):
            if txt[:2]=="S:":
                SSID = txt[2:]
            elif txt[:2]=="P:":
                PWD = txt[2:]
                
        if SSID != None and PWD != None:
        
            play_music("boot.wav")
        
            print("WIFI信息:",SSID,PWD)
            
            cmd = "nmcli r wifi on"
            print("执行命令:",cmd)
            os.system("nmcli r wifi on") #打开wifi
            time.sleep(1) #稍等扫描
            
            cmd = "nmcli dev wifi connect %s password %s"%(SSID,PWD)
            print("执行命令:",cmd)
            os.system(cmd)
            
            print("执行命令结束")
            text = "无线网连接操作完成"
            tts_run(text,tts_method="edge")
        else:
            text = "二维码无效"
            tts_run(text,tts_method="edge")
            time.sleep(3)
            continue
                
                
        
        
        
    
    #print("test 1")
    #tts_run("你好，我是系统语音，请打开收音机",tts_method="local")
    #print("test 2")
    #tts_run("你好，我是百度语音，请打开收音机",tts_method="baidu")
    #print("test 3")

    speakers = [
    ("Xiaoxiao","晓晓"),# (Neural) - 晓晓" Xiaoxiao播放成功
    ("Yunyang","云扬"), # (Neural) - 云扬" Yunyang播放成功
    ("Yunxi","云希"), # (Neural) - 云希" Yunxi播放成功
    ("Yunjian","云健"), # (Neural) - 云健 - 预览" Yunjian播放成功
    ("Yunxia","云夏"),
    ("Xiaoyi","小怡")]# (Neural) - 云夏 - 预览" Yunxia播放成功
'''


say_str_queue = queue.Queue(2)


def sub_tts_callback(msg):
    global say_str_queue
    say_str_queue.put( msg.data )
    
    

if __name__ == "__main__":
    rospy.init_node("tts_listaner")
    
    rospy.loginfo("已开启语音合成节点")
    
    #play_music("boot0.wav")
    
    sub = rospy.Subscriber("tts", String, sub_tts_callback, queue_size=2)
    
    
    tts_run("开始订阅语音话题",tts_method="edge")
    
    rate = rospy.Rate(10) #10Hz 100ms
    
    while not rospy.is_shutdown():

        try:
            say_str = say_str_queue.get(timeout=1) #会阻塞  #等待1秒，如果还是不能取到数据，则抛出异常。
        except:
            rate.sleep() #100ms
            continue
        
        rospy.loginfo("文本转语音:%s",say_str)

        if say_str != "":
            tts_run(say_str,tts_method="edge")
        
        rate.sleep() #100ms
 
