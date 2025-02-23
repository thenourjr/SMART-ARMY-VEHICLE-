import cv2
import urllib.request
import numpy as np
import time
import threading

desired_fps = 20
required_id = 1
url = 'http://192.168.1.13/cam-lo.jpg'
winName = 'ESP32 CAMERA_detection'
cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)

classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
width = 320
height = 240
net.setInputSize(width, height)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

img = None
lock = threading.Lock()
stop_thread = False


def fetch_image():
    global img, stop_thread
    while not stop_thread:
        imgResponse = urllib.request.urlopen(url)
        imgNp = np.array(bytearray(imgResponse.read()), dtype=np.uint8)
        with lock:
            img = cv2.imdecode(imgNp, -1)
        time.sleep(1 / desired_fps)


def trace(classIds, bbox, required_id):
    for idx, id in enumerate(classIds):
        if id == required_id:
            coordinates = bbox[idx]
            x = abs(coordinates[2] - coordinates[0])
            y = abs(coordinates[3] - coordinates[1])
            cx = width // 2
            cy = height // 2
            if abs(x - cx) > 10:
                if x < cx:
                    return "right"
                if x > cx:
                    return "left"
            if abs(y - cy) > 10:
                if y < cy:
                    return "up"
                if y > cy:
                    return "down"
    return "None"


fetch_thread = threading.Thread(target=fetch_image)
fetch_thread.start()

while True:
    start_time = time.time()
    
    with lock:
        if img is not None:
            classIds, confs, bbox = net.detect(img, confThreshold=0.6)
            direction = trace(classIds, bbox, required_id)
            print(direction)
            if len(classIds) != 0:
                for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    if classId != required_id:
                        continue
                    cv2.rectangle(img, box, color=(0, 255, 0), thickness=3)
                    cv2.putText(img, classNames[classId - 1], (box[0] + 10, box[1] + 30),
                                cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow(winName, img)

    processing_time = time.time() - start_time
    delay = max(1, int((1 / desired_fps - processing_time) * 1000))
    tecla = cv2.waitKey(delay) & 0xFF
    if tecla == 27:
        stop_thread = True
        break

cv2.destroyAllWindows()
fetch_thread.join()
