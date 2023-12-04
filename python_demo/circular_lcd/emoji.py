import os
import time

import cv2
import random
from random import randint

from lcd import  Lcd
from uart import Uart

#为了加快读取速度,先将images文件夹移动到开发板本地
root_path = "/home/orangepi/image"

def read_img_dict():
    emoji_states = \
    [
        "眨眼",
        "左上看",
        "右上看",
        "兴奋",
        # "惊恐",
        # "不屑",
        # "愤怒",
        # "难过",
        "图片",
    ]
    imgs_dict = dict()
    for emoji_state in emoji_states:
        if emoji_state=="眨眼":
            paths = [
                os.path.join(root_path, emoji_state, "单次眨眼偶发"),
                os.path.join(root_path, emoji_state, "快速双眨眼偶发")
            ]
        elif emoji_state=="图片":
            paths = [
                os.path.join(root_path, emoji_state, "正常")
            ]
        else:
            paths = [
                os.path.join(root_path,emoji_state,emoji_state+"_1进入姿势"),
                os.path.join(root_path,emoji_state,emoji_state+"_2可循环动作"),
                os.path.join(root_path,emoji_state,emoji_state+"_3回正")
            ]

        for path in paths:
            print("path=",path)
            key = os.path.basename(path) #把文件夹名字作为key
            imgs_dict[key] = []

            for i in range(1,200):
                img_name = os.path.join(path,"%d.jpg"%(i))
                if not os.path.exists(img_name):
                    continue

                img = cv2.imread(img_name)
                imgs_dict[key].append(img)

    # for key,val in imgs_dict.items():
    #     print(key,val)

    return imgs_dict



def do_emoji(uart,lcd,imgs_dict,emoji_state):
    keys = []
    if emoji_state=="眨眼":
        keys.append( (0,0,"单次眨眼偶发"))
        keys.append( (0,0,"快速双眨眼偶发"))
        keys = [random.choice(keys)]  # 随机只挑一个
    elif emoji_state=="图片":
        keys.append( (0,0,"正常"))
    else:
        if  emoji_state=="左上看" or emoji_state=="右上看":
            loop_num = 1
        else:
            loop_num = 4

        if emoji_state=="左上看":
            keys.append((-2000,2000,emoji_state+"_1进入姿势"))
        elif emoji_state=="右上看":
            keys.append((2000,-2000,emoji_state+"_1进入姿势"))
        else:
            keys.append((0, 0, emoji_state+"_1进入姿势"))

        for l in range(loop_num):
            if  emoji_state=="兴奋":
                if l%2==0:
                    keys.append((1500, 1500, emoji_state+"_2可循环动作"))
                elif l%2==1:
                    keys.append((-1500, -1500, emoji_state+"_2可循环动作"))
            else:
                keys.append((0,0,emoji_state+"_2可循环动作"))

        if emoji_state=="左上看":
            keys.append((2000,-2000,emoji_state+"_3回正"))
        elif emoji_state=="右上看":
            keys.append((-2000,2000,emoji_state+"_3回正"))
        else:
            keys.append((0, 0, emoji_state+"_3回正"))

    cnt = 0
    if emoji_state == "兴奋":
        div = 15
    else:
        div = 3

    for pwm1,pwm2,key in keys: #遍历所有key
        imgs = imgs_dict[key]
        imgs_num= len(imgs)

        uart.set_pwm(pwm1,pwm2)
        for i in range(imgs_num):

            if cnt % div == 0: #为加速播放，跳过动画一部分索引的图像
                t1 = time.time()

                lcd.display(imgs[i])#从字典读取图像并显示

                t2 = time.time()
                interval = t2-t1

                if (0.050-interval) >= 0:
                    time.sleep(0.050-interval)
                # else:
                #     print("显示时间:%f ms, 超过了50ms!"%(interval*1000))

            cnt += 1

    uart.set_pwm(0, 0)



if __name__=="__main__":
    lcd = Lcd()
    uart = Uart()
    imgs_dict = read_img_dict()

    while True:
        do_emoji(uart, lcd, imgs_dict, "图片")

        sec = random.uniform(1,4)#停顿1~4秒
        time.sleep( sec )

        emoji_state = random.choice(["眨眼","左上看","右上看","兴奋"])
        do_emoji(uart, lcd, imgs_dict, emoji_state)








