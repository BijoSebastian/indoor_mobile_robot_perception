#!/usr/bin/env python3

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import PoseArray,Pose


def Object_detection_yolo():

    print("LOADING YOLO")

    net = cv2.dnn.readNet("/home/winston/catkin_ws/src/tracking/scripts/yolov4-tiny.cfg", "/home/winston/catkin_ws/src/tracking/scripts/yolov4-tiny.weights")

    #save all the names in file o the list classes

    classes = []

    with open("/home/winston/catkin_ws/src/tracking/scripts/coco.names", "r") as f:

        classes = [line.strip() for line in f.readlines()]

    #get layers of the network

    layer_names = net.getLayerNames()

    #Determine the output layer names from the YOLO model 

    output_layers = [layer_names[i- 1] for i in net.getUnconnectedOutLayers()]

    print("YOLO LOADED")



    cap= cv2.VideoCapture(2)

    while not rospy.is_shutdown():

        ret, img= cap.read()

        # cv2.imshow('cam_view',img)
        # cv2.waitKey(1)

        if not ret:

            break

        image_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        image_pub.publish(image_msg)


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


        

        posearray = PoseArray()

        for i in range(len(boxes)):        

            if i in indexes:

                x, y, w, h = boxes[i]

                pose = Pose()

                label = str(classes[class_ids[i]])

                if class_ids[i] == 0:

                    centers.append([x+w//2, y+h//2])

                    #print(centers)

                    center_x= x+w//2

                    center_y= y+h//2

                    pose.position.x= center_x#-320#Tranforming frame to centre of image

                    pose.position.y= center_y#-240)

                    pose.position.z= 0

                    posearray.poses.append(pose)
                #cv2.rectangle

                print(posearray)
                detection_pub.publish(posearray)


    cap.release()

    cv2.destroyAllWindows()



if __name__=="__main__":

    rospy.init_node("detector")

    bridge= CvBridge()

    #rospy.Subscriber("usb_cam/image_rect_color", Image,callback)

    #Object_detection_yolo()



    detection_pub = rospy.Publisher('detection_topic', PoseArray, queue_size=10)
    image_pub = rospy.Publisher('/cam_image', Image, queue_size=10)



    try:

        Object_detection_yolo()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass

        