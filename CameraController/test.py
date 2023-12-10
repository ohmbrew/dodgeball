import cv2
import numpy as np

img = cv2.imread('redball.jpg')

RED_MIN = np.array([150, 65, 65],np.uint8)
RED_MAX = np.array([180, 255, 255],np.uint8)

hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

frame_threshed = cv2.inRange(hsv_img, RED_MIN, RED_MAX)
cv2.imwrite('output.jpg', frame_threshed)