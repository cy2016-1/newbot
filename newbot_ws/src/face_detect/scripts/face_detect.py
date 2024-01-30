#!/usr/bin/env python3
#coding=utf-8
import time

import rospy
from sensor_msgs.msg import Image

import os
import numpy as np
import cv2
from lcd import Lcd

lcd = Lcd()

current_dir = os.path.dirname(os.path.realpath(__file__))
haarcascades_file_name = os.path.join(current_dir,"haarcascade_frontalface_default.xml")
    
# 进行人脸识别
face_cascade = cv2.CascadeClassifier(haarcascades_file_name)
image_msg = None
def image_callback(msg):
    # 将ROS图像消息转换为OpenCV图像格式
    # 从ROS图像消息中获取图像格式、尺寸和数据
    global image_msg
    image_msg = msg


def main():
    rospy.init_node('face_detect')
    rospy.Subscriber('/camera/image_raw', Image, image_callback)

    rate = rospy.Rate(50)  # 50Hz 20ms

    #rospy.spin()
    while not rospy.is_shutdown():
        if image_msg==None:
            time.sleep(0.1)
            rate.sleep()
            continue

        img_width = image_msg.width
        img_height = image_msg.height
        cv_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape((img_height, img_width, -1))

        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        print("face num=", len(faces))

        # 选择面积最大的人脸
        max_area = 0
        max_face = None
        for (x, y, w, h) in faces:
            area = w * h
            if area > max_area:
                max_area = area
                max_face = (x, y, w, h)

        if max_face is not None:
            x, y, w, h = max_face
            display_image = cv_image[y:y + h, x:x + h, :]
            display_image = cv2.resize(display_image, (240, 240))
        else:
            display_image = np.zeros((240, 240, 3), dtype=np.uint8)  # h,w,c 240x240x3

        lcd.display(display_image,cv2.COLOR_RGB2BGR565)

        rate.sleep()

if __name__ == '__main__':
    main()


