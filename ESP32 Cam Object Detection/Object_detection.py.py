import cv2 #opencv
import urllib.request #to open and read URL
import numpy as np
import socket
import time

TCP_IP = '192.168.1.9'  # ESP32-CAM's IP address
TCP_PORT = 12345  # ESP32-CAM's port
BUFFER_SIZE = 1024

MESSAGE = ""
desired_fps = 120
required_id = 5




#OBJECT CLASSIFICATION PROGRAM FOR VIDEO IN IP ADDRESS

url = 'http://192.168.1.2/cam-lo.jpg'
#url = 'http://192.168.1.6/'
winName = 'ESP32 CAMERA'
cv2.namedWindow(winName,cv2.WINDOW_AUTOSIZE)
#scale_percent = 80 # percent of original size    #for image processing

classNames = []
classFile = 'coco.names'
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
width = 320
height = 320
net.setInputSize(width, height)
# net.setInputSize(480,480)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

def trace(classIds, bbox, required_id):
    """
    This Function takes Classes ids of the objects in the image and their locations then
    trace the object that has id same to required id
    """

    for idx, id in enumerate(classIds):
        if id == required_id:
            coordinates = bbox[idx]
            x = abs(coordinates[2] - coordinates[0])
            y = abs(coordinates[3] - coordinates[1])
            cx = width//2
            cy = height//2
            # print(bbox)
            # print(f"remote:{x,y} , center {cx,cy}")
            if (abs(x-cx) > 10):
                if (x < cx):
                    return "right"
                if (x > cx):
                    return "left"
            if (abs(y-cy) > 10):
                if (y < cy):
                    return "up"
                if (y > cy):             
                    return "down"
                
    return "None"


cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)
while (1):
    start_time = time.time()
    imgResponse = urllib.request.urlopen (url) # here open the URL
    imgNp = np.array(bytearray(imgResponse.read()),dtype=np.uint8)
    img = cv2.imdecode (imgNp,-1) #decodificamos
    classIds, confs, bbox = net.detect(img,confThreshold=0.6)
    # print(classIds, bbox)
    direction = trace(classIds, bbox, required_id)
    print(direction)
    #s.connect((TCP_IP, TCP_PORT))
    direction = direction + "\n"
    s.send(direction.encode())
    #s.close()
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            if classId != required_id:
                continue
            # mostramos en rectangulo lo que se encuentra
            cv2.rectangle(img, box, color=(0, 255, 0), thickness=3)
            cv2.putText(img, classNames[classId-1], (box[0]+10,
                        box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow(winName, img)  # show the picture

    # wait for ESC to be pressed to end the program
    processing_time = time.time() - start_time
    delay = max(1, int((1/desired_fps - processing_time) * 1000))
    tecla = cv2.waitKey(delay) & 0xFF
    if tecla == 27:
        break
s.close()
cv2.destroyAllWindows()