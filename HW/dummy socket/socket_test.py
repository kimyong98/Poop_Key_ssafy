import socket
backend_server_ip = "70.12.246.41"
backend_server_port = 5010
backend_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
backend_socket.connect((backend_server_ip, backend_server_port))

RESTROOM_ID = 1
congestion = 2
backend_socket.sendall(("congestion,%d,%d,"%(congestion, RESTROOM_ID)).encode())
import pdb; pdb.set_trace()
