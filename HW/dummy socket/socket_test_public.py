import socket
import time
public_server_ip = "43.202.37.123"
public_server_port = 8200
public_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
public_socket.connect((public_server_ip, public_server_port))

# RESTROOM_ID = 1
# congestion = 2
# backend_socket.sendall(("congestion,%d,%d,"%(congestion, RESTROOM_ID)).encode())
print("connected")
time.sleep(1)
public_socket.sendall("toiletOccupied,2,-1,".encode())
time.sleep(5)
public_socket.sendall("toiletVacant,2,-1,".encode())
print("wait")


public_socket.close()