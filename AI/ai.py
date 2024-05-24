print("import modules")
from ultralytics import YOLO
import cv2
import threading
import socket
import numpy as np
from shapely.geometry.point import Point
from shapely.geometry import Polygon


print("setting thread")
frame = None
ret = None
state = None
infered_frame = None
is_new_frame = False
end_thread_flag = False
is_full = False


bounding_area = Polygon([(),(),(),()])

server_ip = "0.0.0.0"
server_port = 12345

backend_ip = "0.0.0.0"
backend_port = 12345

frame_lock = threading.Lock()
infered_frame_lock = threading.Lock()
state_lock = threading.Lock()


def read_frames():
    print("start cam thread")
    global frame
    global ret
    global end_thread_flag    

    print("load cam")
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 224)
    cam.set(cv2.CAP_PROP_FPS, 36)
    if(not cam.isOpened()):
        raise Exception("cam is closed")
    
    while True:
        if not is_full :
            continue

        current_ret, current_frame = cam.read()
        with frame_lock:
            if current_ret :
                frame = current_frame[:, :, [2, 1, 0]]
            ret = current_ret

        if end_thread_flag :
            break
    cam.release()
    print("end cam thread")

def check_object():
    print("start ai thread")
    global state
    global frame
    global infered_frame
    global end_thread_flag    
    global is_new_frame

    print("load model")
    model = YOLO("yolov8n.pt")
    while True:
        if not is_full :
            continue

        with frame_lock:            
            current_frame = frame            
        if current_frame is None:
            continue
        results = model.predict(current_frame, imgsz=(224,224), device = "cpu", classes = 0)
        with infered_frame_lock:
            infered_frame = results[0]
            is_new_frame = True
        if end_thread_flag :
            break
    print("end ai thread")

def send_congestion():
    global is_new_frame
    global frame
    global client_socket
    while True:
        if not is_full :
            continue
        with frame_lock:
            this_frame = infered_frame
            is_updated = is_new_frame            
            if is_new_frame :
                is_new_frame = False
        if is_updated :
            is_crowded = False
            for box in this_frame.boxes:
                bbox_center = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                if (bounding_area.contains(Point(bbox_center[0], bbox_center[1]))) :
                    is_crowded = True
                    break
            
            if is_crowded:
                client_socket.sendall("1".encode())
                print("send to stm: 1")
            else:
                client_socket.sendall("0".encode())
                print("send to stm: 0")
        else:
            continue


    
cam_thread = threading.Thread(target=read_frames)
ai_thread = threading.Thread(target=check_object)
socket_thread = threading.Thread(target = send_congestion)
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
backend_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


print("connecting to backend...")
backend_socket.connect((server_ip, server_port))
print("backend connected!")


server_socket.bind((server_ip, server_port))
server_socket.listen()
print("wait for client...")
client_socket, addr = server_socket.accept()
print("client is connected!")

socket_thread.start()
ai_thread.start()
cam_thread.start()
while(True):
    received = client_socket.recv(1024).decode().strip()
    if not received :
        print("client disconnected")
        is_full = False
        client_socket.close()
        with infered_frame_lock:
            is_new_frame = False

        server_socket.listen()
        print("wait for client...")
        client_socket, addr = server_socket.accept()
        print("client is connected!")        

    elif received == "is full":        
        if_full = True

    elif received == "is not full":        
        is_full = False

    elif received[:14] == "toiletOccupied" or received[:12] == "toiletVacant" or received[:12] == "tissueStatus" or received[:11] == "toiletBreak": 
        backend_socket.sendall(received.encode())
        print("send to backend: %s"%received)

    else:
        print("get wrong data: %s"%(received))
        
