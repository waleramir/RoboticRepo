import cv2
import numpy as np


cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)
low_black = np.array([90,50,50])
high_black = np.array([130,255,255])
while True:
    ret, image = cap.read()
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    black_mask = cv2.inRange(hsv_image, low_black, high_black)
    contours, hierarchy = cv2.findContours(
        black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    for contour in contours:
        area = cv2.contourArea(contour)
        M = cv2.moments(contour)
        if area > 90:
            cv2.drawContours(image, contour, -1, (0, 255, 0), 2)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
               
                #print("x ", cx)
                print("area ", area)
                cv2.circle(image, (cx, cy), radius=2, color=(0, 0, 255), thickness=-1)
    cv2.imshow("Video", image)
    k = cv2.waitKey(1)
    if k == 27:
        break
cap.release()