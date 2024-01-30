import os
import asyncio
import edge_tts
import pyttsx3
import socket
import speech_recognition as sr #pip install SpeechRecognition
from aip import AipSpeech
import database

class EdgeTTS():
    def __init__(self, voice="zh-CN-XiaoxiaoNeural"):
        self.voice = voice
        self.db = database.DataBase()

    async def async_get_speech(self, text):
        try:
            #mp3_name = text+".mp3" #如果不写绝对路径，则会保存在~/.ros/目录下
            tts = edge_tts.Communicate(text=text, voice=self.voice)
            #await tts.save(mp3_name)
            mp3_data = b""
            async for message in tts.stream():
                if message["type"] == "audio":
                    mp3_data += message["data"]
                else:
                    pass #单词offset数据不需要，跳过
            return mp3_data
        except Exception as e:
            print("edge tts合成失败")
            return None

    def text_to_speech(self, text):
        #mp3_name = text + ".mp3"  # 如果不写绝对路径，则会保存在~/.ros/目录下
        # if os.path.exists(mp3_name):  # 如果存在，则直接返回
        #     print("命中已存在文件:", mp3_name)
        #     return mp3_name

        mp3_data = self.db.get_audio(text)
        if mp3_data!=None:
            print("数据条数:",self.db.get_count(),"命中数据库:",text)
            return mp3_data

        event_loop = asyncio.new_event_loop()
        mp3_data = event_loop.run_until_complete(self.async_get_speech(text))
        event_loop.close()
        if mp3_data!=None:#在线生成的数据正常
            self.db.save_audio(text,mp3_data)#如果没有命中，则保存数据库
            print("数据条数:",self.db.get_count(),"未命中，已在线生成并保存数据库")

        return mp3_data


class LocalTTS():
    def __init__(self):
        pass

    def text_to_speech(self, text):
        engine = pyttsx3.init()
        engine.setProperty('voice', 'zh')  # 开启支持中文
        engine.say(text)
        engine.runAndWait()
        return None



def create_tts(type):
    if type == "edge":
        tts_client = EdgeTTS()
    elif type == "local":
        tts_client = LocalTTS()
    else:
        print("tts type error!")
        exit(-1)

    return tts_client