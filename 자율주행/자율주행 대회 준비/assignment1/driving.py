#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random



Offset = 330



def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)   
    return img

def onMouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("point : ", x, y)

def roi_l(image):
    polygons = np.array([[(100,350),(380,350),(160,420),(1,420)]])
    image_mask = np.zeros_like(image)
    cv2.fillPoly(image_mask,polygons,255)
    masking_image = cv2.bitwise_and(image,image_mask)
    return masking_image

def roi_r(image):
    polygons = np.array([[(360,350),(530,350),(635,420),(460,420)]])
    image_mask = np.zeros_like(image)
    cv2.fillPoly(image_mask,polygons,255)
    masking_image = cv2.bitwise_and(image,image_mask)
    return masking_image

def roi(image):
    triangle = np.array([[(5,407),(630,420),(320,264)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask,triangle,255)
    mask = cv2.bitwise_and(image,mask)
    return mask

def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            cv2.line(lines_image, (x1, y1), (x2, y2), (255,0,0), 10)
    return lines_image

def display(image, lines):
    lines_image = np.zeros_like(image)
    if lines is not None:
        for i in xrange(len(lines)):
            for x1, y1, x2, y2 in lines[i]:
                cv2.line(lines_image, (x1,y1), (x2,y2), (255,0,0), 10)
    return lines_image

def make_points(image, average):
    slope, y_int = average
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1-y_int)/slope)
    x2 = int((y2-y_int)/slope)
    return np.array([x1,y1,x2,y2])

def average(image, lines):
    left = []
    right = []

    for line in lines:
        # print(line)
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1,x2), (y1,y2), 1)
        slope = parameters[0]
        y_int = parameters[1]

        if slope < 0:
            left.append((slope, y_int))
        else:
            right.append((slope,y_int))

    rigth_avg = np.average(right, axis = 0)
    left_avg = np.average(left, axis = 0)

    left_line = make_points(image, left_avg)
    right_line = make_points(image, rigth_avg)
    return np.array([left_line, right_line])


#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

def mask_img(image, blue_threshold = 200, green_threshold = 200, red_threshold = 200):
    bgr_threshold = [blue_threshold, green_threshold, red_threshold]
    thresholds = (image[:,:,0] < bgr_threshold[0]) \
                | (image[:,:,1] < bgr_threshold[1]) \
                | (image[:,:,2] < bgr_threshold[2]) 
    image[thresholds] = [0,0,0]
    return image

def process_image(frame):
    global Offset
    lpos = 70
    rpos = 590

    # masking only white color 
    masking_tmp = frame.copy() 
    mask = mask_img(masking_tmp)
    # cv2.imshow("mask_img", mask)
    
    src = cv2.cvtColor(mask,cv2.COLOR_BGR2GRAY)
    dst = cv2.Canny(src, 50,200,None, 3)

    dst_l = roi_l(dst)
    dst_r = roi_r(dst)
    # cv2.imshow("roi", dst_l)
    cdst_l = cv2.cvtColor(dst_l, cv2.COLOR_GRAY2BGR)
    cdst_r = cv2.cvtColor(dst_r, cv2.COLOR_GRAY2BGR)
    cdstp_l = np.copy(cdst_l)
    cdstp_r = np.copy(cdst_r)

    # dst_m = roi(dst)
    
    # cdstp = cv2.cvtColor(dst_m,cv2.COLOR_GRAY2BGR)


    ############################# houghlines
    # lines= cv2.HoughLines(dst, 1, np.pi / 180, 150, None, 0,0)
    # if lines is not None:
    #     for i in range(0, len(lines)):
    #         rho = lines[i][0][0]
    #         theta = lines[i][0][1]
    #         a= math.cos(theta)
    #         b = math.cos(theta)
    #         x0 = a * rho
    #         y0 = b * rho
    #         pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
    #         pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
    #         cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    ######################################################

    ########################### roi divide left, right 
    linesP_l = cv2.HoughLinesP(dst_l, 1, np.pi / 180, 50, None, 50, 10)
    linesP_r = cv2.HoughLinesP(dst_r, 1, np.pi / 180, 50, None, 50, 10)
    ###############################

    ############################ roi create only one 
    # linesP = cv2.HoughLinesP(dst_m, 1, np.pi / 180, 20, 30, 40)
    # test_lines = display(frame, linesP)
    # cv2.imshow("test_img", test_lines)



    if linesP_l is not None:
        for line in linesP_l:
            for x1, y1, x2, y2 in line:
                slope = (y2-y1) / (x2-x1)
                if(slope) <= 0:
                    cv2.line(cdstp_l, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)
                    lpos = (x1 + x2) /2
    
    if linesP_r is not None:
        for line in linesP_r:
            for x1, y1, x2, y2 in line:
                slope = (y2-y1) / (x2-x1)
                if(slope) >= 0:
                    cv2.line(cdstp_r, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)
                rpos = (x1 + x2 )/ 2


    # average_lines = average(frame, linesP)
    # black_lines = display_lines(frame, average_lines)
    
    # cv2.imshow("frame", black_lines)
    # line_image = np.zeros_like(frame)
    # if average_lines is not None:
    #     for line in average_lines:
    #         x1, y1, x2, y2 = line
    #         cv2.line(line_image, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)

    
    # cv2.imshow("left line show", cdstp_l)
    # cv2.imshow("right line show", cdstp_r)

    cdstp_add =cv2.add(cdstp_l, cdstp_r)
    combo_image = cv2.addWeighted(frame, 0.8, cdstp_add,1,1)
    # cv2.imshow("line show", cdstp_add)
    #cv2.imshow("lines", line_image)
    #print(x1," ", y1," ",x2, " ",y2)
    # print("lpos =", lpos, "rpos = ", rpos)

    combo_image = draw_rectangle(combo_image, lpos, rpos, offset=Offset)

    return (lpos, rpos), combo_image





#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================
def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():

        # 이미지처리를 위해 카메라 원본이미지를 img에 복사 저장
        img = image.copy()  
        pos, frame = process_image(img)
        

        # 디버깅을 위해 모니터에 이미지를 디스플레이
        cv2.imshow("CAM View", frame)
        cv2.setMouseCallback('CAM View', onMouse)
        cv2.waitKey(1)       
       
        #=========================================
        # 핸들조향각 값인 angle값 정하기.
        # 차선의 위치 정보를 이용해서 angle값을 설정함.        
        #=========================================

        # 우선 테스트를 위해 직진(0값)으로 설정
        speed = 10

        angle = (pos[0] +pos[1])/2 -320
        print(angle)
        angle = angle * (0.228)
        print(angle)
        if abs(angle) < 6.5:
            speed = 14
        else :
            speed = 10

        # print(pos[0], pos[1], angle)
	    #angle = (pos[0] +pos[1]) / 2 * (0.15)
        #print(angle)
        
        #=========================================
        # 차량의 속도 값인 speed값 정하기.
        # 직선 코스에서는 빠른 속도로 주행하고 
        # 회전구간에서는 느린 속도로 주행하도록 설정함.
        #=========================================

        # 우선 테스트를 위해 느린속도(10값)로 설정
        
		
        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

