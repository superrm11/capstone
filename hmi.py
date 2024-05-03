import socket
import time
import cv2 as cv

HOST = 'localhost'
PORT = 1123
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Wait for a connection to the robot

while True:
    
    # Wait for a connection to the robot
    while True:
        try:
            client.connect((HOST, PORT))
            print("Connected!")
            break
        except:
            print("Failed to connect to the jetson, retrying in 1 second...")
            time.sleep(1)
            
    while True:
        print("Waiting for Jetson to be ready...")
        try:
            while client.recv(1024) != b"READY": pass
        except ConnectionResetError:
            print("Connection reset, retrying...")
            break
        
        print("Ready! Press SPACE to start automated program.")
        while cv.waitKey() != ord(' '): time.sleep(0.2)
        client.sendmsg("BEGIN")
                    
    
    print("HMI Server closed. Restarting...")