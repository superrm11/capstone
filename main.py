#!/bin/python3
import cv2 as cv
from visionops import process

vc = cv.VideoCapture(0)

process(vc)
