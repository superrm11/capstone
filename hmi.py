import socket
import time
import cv2 as cv

HOST = ''
PORT = ''

while True:
    s = None
    print("Waiting for clients...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()
        with conn:
            print("Connected to " + addr)
            while True:
                print("Waiting for Jetson to be ready...")
                data = conn.recv(1024)
                while data != "READY": time.sleep(0.2)
                
                print("Ready! Press SPACE to start automated program.")
                while cv.waitKey() != ord(' '): time.sleep(0.2)
                    
    
    print("HMI Server closed. Restarting...")