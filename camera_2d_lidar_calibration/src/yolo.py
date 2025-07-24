import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import numpy as np


def apply_bounding_boxes(image, center_x, center_y, width, height):
    # Calculate the top-left and bottom-right coordinates of the bounding box
    x1 = int(center_x - width / 2)
    y1 = int(center_y - height / 2)
    x2 = int(center_x + width / 2)
    y2 = int(center_y + height / 2)

    # Draw the bounding box on the image
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    return image


def Object_detection_yolo(img):

    cam_detections=[]
    height= img.shape[0]

    width= img.shape[1]

    blob = cv2.dnn.blobFromImage(img, 1 / 255.0, (416, 416),

    swapRB=True, crop=False)

    #Detecting objects

    net.setInput(blob)

    outs = net.forward(output_layers)

    # Showing informations on the screen

    class_ids = []

    confidences = []

    boxes = []

    for out in outs:

        for detection in out:

            scores = detection[5:]

            class_id = np.argmax(scores)

            confidence = scores[class_id]

            if confidence > 0.5:

                # Object detected

                center_x = int(detection[0] * width)

                center_y = int(detection[1] * height)

                w = int(detection[2] * width)

                h = int(detection[3] * height)



                # Rectangle coordinates

                x = int(center_x - w / 2)

                y = int(center_y - h / 2)



                boxes.append([x, y, w, h])

                confidences.append(float(confidence))

                class_ids.append(class_id)



    #We use NMS function in opencv to perform Non-maximum Suppression

    #we give it score threshold and nms threshold as arguments.

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    centers= []


    for i in range(len(boxes)):        

        if i in indexes:

            x, y, w, h = boxes[i]

            

            label = str(classes[class_ids[i]])

            if class_ids[i] == 0:

                centers.append([x+w//2, y+h//2])

                center_x= x+w//2

                center_y= y+h//2

                cam_detections.append([center_x,center_y])
            
    for i in cam_detections:
        try:
            img=apply_bounding_boxes(img, center_x, center_y, w, h)
            cv2.circle(img, (int(round(i[0])),int(round(i[1]))), 10, (255,0,0), -1)
        except Exception as err:
            print(err)
            continue

    return img,cam_detections


bridge = CvBridge()

net = cv2.dnn.readNet("/home/winston/catkin_ws/src/indoor_mobile_robot_perception/camera_2d_lidar_calibration/src/yolov4-tiny.cfg","/home/winston/catkin_ws/src/indoor_mobile_robot_perception/camera_2d_lidar_calibration/src/yolov4-tiny.weights")

#save all the names in file of the list classes

classes = []

with open("/home/winston/catkin_ws/src/indoor_mobile_robot_perception/camera_2d_lidar_calibration/src/coco.names", "r") as f:

    classes = [line.strip() for line in f.readlines()]

#get layers of the network

layer_names = net.getLayerNames()

#Determine the output layer names from the YOLO model 

output_layers = [layer_names[i- 1] for i in net.getUnconnectedOutLayers()]

print("YOLO LOADED")


# Capture video from webcam
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    img,cam_detections=Object_detection_yolo(frame)
    print(cam_detections)


    # Display the frame with foreground
    cv2.imshow('yolo', img)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close windows
cap.release()
cv2.destroyAllWindows()
