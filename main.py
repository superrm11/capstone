#!/bin/python3
import cv2 as cv
from visionops import process

vc = cv.VideoCapture(0)

while(cv.waitKey(5) != ord('q')):
    arr = process(vc)
    if (len(arr) > 0):
        print(arr)
