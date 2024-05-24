import socket
import os
import threading
ai_server_ip = '192.168.0.41'
ai_server_port = 12345
backend_server_ip = ""
backend_server_port = 5050

RESTROOM_ID = 1

is_crowded = False
is_full = False
is_AI_counting = False
before_congestion = -1

# 소켓 객체 생성
AI_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
Backend_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def uart_interupt():
    global is_crowded
    global AI_socket
    global is_AI_counting
    while True:
        if not is_AI_counting :
            continue
        data = AI_socket.recv(1024).decode().strip()
        if(data == "is crowded") :
            is_crowded = True
        elif (data == "is not crowded"):
            is_crowded = False

def check_toilet_is_full():
    if os.listdir("./toliet").count >= 1:
        return True
    else :
        return False

def send_congestion():
    global is_full
    global is_crowded
    global before_congestion
    if(not is_full) :
        congestion = 0
    elif(is_full and not is_crowded) :
        congestion = 1
    elif(is_full and is_crowded):
        congestion = 2
    else :
        raise Exception("logic error")
    if(congestion != before_congestion):        
        Backend_socket.sendall("congestion,%d,%d,"%(congestion, RESTROOM_ID))
        before_congestion = congestion


uart_IRC = threading.Thread(target=uart_interupt)

Backend_socket.connect((backend_server_ip, backend_server_port))
while True:
    is_full = check_toilet_is_full()
    if is_full :
        if not is_AI_counting :
            AI_socket.connect((ai_server_ip, ai_server_port))
            print("connected to server")
            AI_socket.sendall("is full")
            print("send count request")
            is_AI_counting = True
    else:
        if is_AI_counting :
            AI_socket.sendall("is not full")
            print("send stop request")
            AI_socket.close()
            print("close socket")
            is_AI_counting = False
    
    send_congestion()
