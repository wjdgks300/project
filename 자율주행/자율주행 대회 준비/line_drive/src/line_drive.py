#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time

Width = 640
Height = 480
Offset = 330

# draw rectangle
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
    polygons = np.array([[(8,370),(140,380),(200,320),(35,320)]])
    image_mask = np.zeros_like(image)
    cv2.fillPoly(image_mask,polygons,255)
    masking_image = cv2.bitwise_and(image,image_mask)
    return masking_image

def roi_r(image):
    polygons = np.array([[(470,370),(630,370),(570,320),(470,320)]])
    image_mask = np.zeros_like(image)
    cv2.fillPoly(image_mask,polygons,255)
    masking_image = cv2.bitwise_and(image,image_mask)
    return masking_image



# You are to find "left and right position" of road lanes
def process_image(frame):
    global Offset
    lpos = 0
    rpos = 640
    #
    src = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    dst = cv2.Canny(src, 50,200,None, 3)

    dst_l = roi_l(dst)
    dst_r = roi_r(dst)
    # cv2.imshow("roi", dst_l)
    cdst_l = cv2.cvtColor(dst_l, cv2.COLOR_GRAY2BGR)
    cdst_r = cv2.cvtColor(dst_r, cv2.COLOR_GRAY2BGR)
    cdstp_l = np.copy(cdst_l)
    cdstp_r = np.copy(cdst_r)

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
    
    linesP_l = cv2.HoughLinesP(dst_l, 1, np.pi / 180, 50, None, 50, 10)
    linesP_r = cv2.HoughLinesP(dst_r, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP_l is not None:
        for line in linesP_l:
            for x1, y1, x2, y2 in line:
                cv2.line(cdstp_l, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)
                lpos = (x1 + x2) /2
    
    if linesP_r is not None:
        for line in linesP_r:
            for x1, y1, x2, y2 in line:
                cv2.line(cdstp_r, (x1,y1),(x2,y2), (0,0,255),3,cv2.LINE_AA)
                rpos = (x1 + x2 )/ 2

    cv2.imshow("left line show", cdstp_l)
    cv2.imshow("right line show", cdstp_r)
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    return (lpos, rpos), frame


def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 1.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res
    cv2.imshow('steer', image)
    

# You are to publish "steer_anlge" following load lanes
if __name__ == '__main__':
    cap = cv2.VideoCapture('kmu_track.mkv')
    time.sleep(3)

    while not rospy.is_shutdown():
        ret, image = cap.read()
        pos, frame = process_image(image)
        steer_angle = (pos[0] + pos[1])/2 * (0.15)
        
    
        draw_steer(frame, -(steer_angle - 50))

        cv2.setMouseCallback('steer', onMouse)
        
        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

