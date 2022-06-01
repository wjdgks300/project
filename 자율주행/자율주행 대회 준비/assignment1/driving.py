#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# �Բ� ���Ǵ� ���� ���̽� ��Ű������ import �����
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
    polygons = np.array([[(120,350),(390,350),(190,420),(1,420)]])
    image_mask = np.zeros_like(image)
    cv2.fillPoly(image_mask,polygons,255)
    masking_image = cv2.bitwise_and(image,image_mask)
    return masking_image

def roi_r(image):
    polygons = np.array([[(250,350),(500,350),(635,420),(340,420)]])
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

def average(lines):
    if len(lines) == 0:
        return
    elif len(lines) == 1:
        return lines

    num_lines = len(lines)

    sum_x1 = 0
    sum_y1 = 0
    sum_x2 = 0
    sum_y2 = 0

    for line in lines:
        for x1, y1, x2, y2 in line:
            sum_x1 += x1
            sum_y1 += y1
            sum_x2 += x2
            sum_y2 += y2

    avg_line = np.array([sum_x1/num_lines,sum_y1/num_lines,sum_x2/num_lines,sum_y2/num_lines])
    return avg_line


#=============================================
# �͹̳ο��� Ctrl-C Ű�Է����� ���α׷� ������ ���� ��
# �� ó���ð��� ���̱� ���� �Լ�
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# ���α׷����� ����� ��� �����
#=============================================
image = np.empty(shape=[0]) # ī�޶� �̹����� ���� ����
bridge = CvBridge() 
motor = None # ���� ������ ���� ����

#=============================================
# ���α׷����� ����� ��� �����
#=============================================
CAM_FPS = 30    # ī�޶� FPS - �ʴ� 30���� ������ ����
WIDTH, HEIGHT = 640, 480    # ī�޶� �̹��� ����x���� ũ��

#=============================================
# �ݹ��Լ� - ī�޶� ������ ó���ϴ� �ݹ��Լ�
# ī�޶� �̹��� ������ �����ϸ� �ڵ����� ȣ��Ǵ� �Լ�
# ���ȿ��� �̹��� ������ ���� image ������ �Ű� ����.
#=============================================
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# ���� ������ �����ϴ� �Լ�  
# �Է����� ���� angle�� speed ���� 
# ���� ���ȿ� �Ű� ���� �Ŀ� ������ ������.
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
    lpos = 60
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
    linesP_l = cv2.HoughLinesP(dst_l, 1, np.pi / 180, 40, None, 50, 10)
    linesP_r = cv2.HoughLinesP(dst_r, 1, np.pi / 180, 40, None, 50, 10)
    ###############################

    ############################ roi create only one 
    # linesP = cv2.HoughLinesP(dst_m, 1, np.pi / 180, 20, 30, 40)
    # test_lines = display(frame, linesP)
    # cv2.imshow("test_img", test_lines)

    
    avg_lines_l = average(linesP_l)
    avg_lines_r = average(linesP_r)
    print(avg_lines_l)
    print(avg_lines_r)
    # print(linesP_l[0], linesP_l[1])

    # print(len(linesP_l))
    if linesP_l is not None:
        for line in linesP_l:
            for x1, y1, x2, y2 in line:
                slope = float(y2-y1) / float(x2-x1)
                # print(slope)
                if(slope) <= 0:
                    cv2.line(cdstp_l, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)
                    lpos = (x1 + x2) /2
    
    if linesP_r is not None:
        for line in linesP_r:
            for x1, y1, x2, y2 in line:
                slope = float(y2-y1) / float(x2-x1)
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
# �������� ���� �Լ� 
# ī�޶� ������ �޾� ���� ����ó���� �˰����� ����
# ������ ��ġ�� �ľ��� �Ŀ� ���Ⱒ�� �����ϰ�,
# ���������� ���� ������ �����ϴ� ���� ������. 
#=============================================
def start():

    # ������ ������ ������ start() �ȿ��� ����ϰ��� ��
    global motor, image

    #=========================================
    # ROS ��带 �����ϰ� �ʱ�ȭ ��.
    # ī�޶� ������ �����ϰ� ���� ������ ������ ������ ����
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # ù��° ī�޶� ������ ������ ������ ��ٸ�.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    #=========================================
    # ���� ���� 
    # ī�޶� ������ �����ϴ� �ֱ⿡ ���� �ѹ��� ������ ���鼭 
    # "�̹���ó�� +������ġã�� +���Ⱒ���� +�������ȹ���" 
    # �۾��� �ݺ������� ������.
    #=========================================
    while not rospy.is_shutdown():

        # �̹���ó���� ���� ī�޶� �����̹����� img�� ���� ����
        img = image.copy()  
        pos, frame = process_image(img)
        

        # ������� ���� ����Ϳ� �̹����� ���÷���
        cv2.imshow("CAM View", frame)
        cv2.setMouseCallback('CAM View', onMouse)
        cv2.waitKey(1)       
       
        #=========================================
        # �ڵ����Ⱒ ���� angle�� ���ϱ�.
        # ������ ��ġ ������ �̿��ؼ� angle���� ������.        
        #=========================================

        # �켱 �׽�Ʈ�� ���� ����(0��)���� ����
        speed = 10

        angle = (pos[0] +pos[1])/2 -320
        # print(angle)
        angle = angle * (0.228)
        # print(angle)
        if abs(angle) < 1.5:
            speed = 16
        if abs(angle) < 6.5:
            speed = 14
        elif abs(angle) > 6.5 and abs(angle) <12:
            speed = 10
        elif abs(angle) < 14:
            speed = 7
            angle = angle * (1.2)
        else :
            speed = 5
            angle = angle * (1.35)

        # print(pos[0], pos[1], angle)
	    #angle = (pos[0] +pos[1]) / 2 * (0.15)
        #print(angle)
        
        #=========================================
        # ������ �ӵ� ���� speed�� ���ϱ�.
        # ���� �ڽ������� ���� �ӵ��� �����ϰ� 
        # ȸ������������ ���� �ӵ��� �����ϵ��� ������.
        #=========================================

        # �켱 �׽�Ʈ�� ���� �����ӵ�(10��)�� ����
        
		
        # drive() ȣ��. drive()�Լ� �ȿ��� ���� ������ �����.
        drive(angle, speed)


#=============================================
# ���� �Լ�
# ���� ���� ȣ��Ǵ� �Լ��� ���⼭ start() �Լ��� ȣ����.
# start() �Լ��� �������� ���� �Լ���. 
#=============================================
if __name__ == '__main__':
    start()

