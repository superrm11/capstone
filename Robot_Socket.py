import socket

# Define the IP address and port to connect to
HOST = '127.0.0.1'
PORT = 5024

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
client_socket.connect((HOST, PORT))

# Send data to the server
data_to_send = "Hello from Python!"
client_socket.send(data_to_send.encode())

# Receive data from the server
received_data = client_socket.recv(1024).decode()
print("Received data from RobotStudio:", received_data)

# Close the socket
client_socket.close()
