#!/bin/python3
from asyncio import wait_for
import socket
import time
from typing import List
import cv2 as cv
from visionops import process
import qwiic_vl53l1x

# TODO 
# - Serial communication to Arduino 
# - Arduino code (receive command, drive stepper)
# - Generate map function from list of contours
# - Client program - Spacebar to command start, socket to send commands / receive map

HOST = '127.0.0.1'
PORT = 5024
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

while True:
    try:
        client.connect((HOST, PORT))
        break
    except:
        print("Failed to connect, retrying in 1 second...")

    time.sleep(1)
print("Client Connected.")

def send(cmd):
    print("Sent \"" + cmd + "\" command to Jetson")
    client.send(cmd.encode())
def recv():
    retval = ""
    try:
        retval = client.recv(1024).decode()
    except:
        pass

    return retval

# 1 = error
# 0 = OK
def wait_until(condition, timeout, interval):
    start = time.time()
    while not condition():
        end = time.time()
        if(end - start > timeout):
            return 1
    0
        
def wait_for_ok():
    print("Waiting for OK signal...")
    ret = wait_until(lambda: recv() == "OK", 5, 0.1)
    if(ret == 1):
        print("Communication Error!")
        exit(1)
    print("OK received. Continuing...")

def wait_for_done():
    print("Waiting for DONE signal...")
    ret = wait_until(lambda: recv() == "DONE", 10, 0.1)
    if(ret == 1):
        print("Watchdog error - robot took too long!")
        exit(1)
    print("OK received. Continuing...")

def get_lidar_mm():
    lidar.start_ranging()
    time.sleep(.005)
    print(lidar.get_distance())
    time.sleep(.005)
    lidar.stop_ranging()
        
# Initialize Camera (jetson CSI port)
vc = cv.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")

# Initialize LIDAR (i2c 1, address 0x52)
lidar = qwiic_vl53l1x.QwiicVL53L1X()
retval = lidar.sensor_init()
if retval == None:
    print("Lidar initialized")
else:
    print("Failed to initialize Lidar")
lidar.set_distance_mode(1)


while True:
    # Step 0 - Wait for HMI command to start!
    # Step 1 - take initial picture
    print("Taking picture...")
    send("PIC") # Tell jetson to go to "picture position"
    wait_for_ok()
    wait_for_done()

    defect_list = process(vc) # Take the picture & process for possible defects

    dist_list = []
    # Step 2 - Scan each point of interest for lidar depth
    for poi in defect_list:
        send("LGOTO " + poi[0] + " " + poi[1])
        wait_for_ok()
        wait_for_done()
        # Get lidar value
        d = get_lidar_mm()
        dist_list.append(d) #TODO offset needed
        print("LIDAR distance: " + d + "mm")
        


while(cv.waitKey(5) != ord('q')):
    arr = process(vc)
    if (len(arr) > 0):
        print(arr)
