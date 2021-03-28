import cv2
import numpy as np
cap = cv2.VideoCapture('color_ball.mp4')
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(
    'M', 'J', 'P', 'G'), 10, (640, 480))
low_yellow = np.array([20, 110, 110])
high_yellow = np.array([40, 255, 255])
while True:
    ret, image = cap.read()
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv_image, low_yellow, high_yellow)
    contours, hierarchy = cv2.findContours(
        yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_L1)
    for contour in contours:
        area = cv2.contourArea(contour)
        M = cv2.moments(contour)

        if area > 6000:
            cv2.drawContours(image, contour, -1, (0, 255, 0), 2)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print(cx)
                cv2.circle(image, (cx,cy), radius=2, color=(0, 0, 255), thickness=-1)
    cv2.imshow('Video', image)
    out.write(image)
    k = cv2.waitKey(1)
    if k == 27:
        break
cap.release()
out.release()
cv2.destroyAllWindows()