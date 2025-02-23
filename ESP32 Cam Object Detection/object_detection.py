import cv2  # opencv
import urllib.request  # to open and read URL
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
# -------------------------------------------------------------------------------
classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
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

print("started")
winName = "detection"


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
    pass


cv2.namedWindow(winName, cv2.WINDOW_AUTOSIZE)

while (1):
    _, frame = cap.read()
    classIds, confs, bbox = net.detect(frame, confThreshold=0.55)
    # print(classIds, bbox)
    required_id = 77
    direction = trace(classIds, bbox, required_id)
    print(direction)
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            if classId != required_id:
                continue
            # mostramos en rectangulo lo que se encuentra
            cv2.rectangle(frame, box, color=(0, 255, 0), thickness=3)
            cv2.putText(frame, classNames[classId-1], (box[0]+10,
                        box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow(winName, frame)  # show the picture

    # wait for ESC to be pressed to end the program
    tecla = cv2.waitKey(5) & 0xFF
    if tecla == 27:
        break
cv2.destroyAllWindows()
