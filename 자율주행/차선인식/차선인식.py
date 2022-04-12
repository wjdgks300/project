import cv2
import sys
import math
import cv2 as cv
import numpy as np


cap = cv2.VideoCapture("video/video.mp4")
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(width, height, end=',')

while(True):
    ret, src = cap.read()
    src = cv2.resize(src, (1280, 720))
    #resize error 가 남 왜 나는지 찾아봐야 한다.
    dst = cv.Canny(src, 50, 200, None, 3)
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)

    lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
    #허프 변환 cv2.HoughLines(image, rho, theta, threshold[, lines[, srn[, stn[, min_theta[, max_theta]]]]])
    #rho - r값의 범위
    # theta - 세타값의 범위
    # thershold - 숫자가 적으면 많은 선이 검출되지만 정확도가 떨어지고 올라가면 선이 적게 나올 수 있음
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.cos(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)

    linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0],l[1]), (l[2], l[3]), (0,0,255),3,cv.LINE_AA)

    cv.imshow("source" ,src)
    cv.imshow("hough line", cdst)
    cv.imshow("Probabilistic line transform", cdstP)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cap.release()
    cv2.destroyAllWindows()
