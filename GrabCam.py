import cv2 as cv
CAMERA = cv.VideoCapture(5)  
s0, img0 = CAMERA.read()
cv.imshow('FOI',img0)
cv.waitKey(0)