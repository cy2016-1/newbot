import speech_recognition as sr #pip install SpeechRecognition
from aip import AipSpeech
import os

APP_ID = os.environ.get('BAIDU_APPID')     #填写控制台中获取的 APPID 信息
API_KEY = os.environ.get('BAIDU_APIKEY')    #填写控制台中获取的 APIKey 信息
SECRET_KEY = os.environ.get('BADIU_APISECRET')   #填写控制台中获取的 APISecret 信息
if APP_ID==None or API_KEY==None or SECRET_KEY==None:
    print("Please set environment variables: BAIDU_APPID,BAIDU_APIKEY,BADIU_APISECRET!!!")
    os._exit(-1)

# 读取文件
def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()

class BaiduASR():
    def __init__(self, appid, api_key, api_secret, **args):
        super(self.__class__, self).__init__()
        self.appid = appid
        self.api_key = api_key
        self.api_secret = api_secret
        self.client = AipSpeech(appid, api_key, api_secret)

    def audio_to_text(self, wav_data):
        #res = self.client.asr(wav_data, 'wav', 16000, {'dev_pid': 1537, })
        res = self.client.asr(wav_data, 'pcm', 16000, {'dev_pid': 1537, }) # 普通话(纯中文识别)
        #print("res=",res)
        if 'result' in res:
            res = res['result'][0]
        else:
            res = ""

        return res

def create_asr(type):
    if type == "baidu":
        asr_client = BaiduASR(APP_ID, API_KEY, SECRET_KEY)
    else:
        print("asr type error!")
        exit(-1)

    return asr_client



