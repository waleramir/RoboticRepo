import numpy as np
import cv2

img_src= cv2.imread('best-in-national-costume.jpg')
kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
kernel2 = np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]])
kernel3 = np.array([[1, 2, 1], [2, 4, 2], [1, 2, 1]])/16
img_rst= cv2.filter2D(img_src,-1,kernel)
img_rst2= cv2.filter2D(img_src,-1,kernel3)

cv2.imshow('Original',img_src)
cv2.imshow('Result',img_rst)
cv2.imshow('Result2',img_rst2)
cv2.waitKey(0)
cv2.destroyAllWindows()