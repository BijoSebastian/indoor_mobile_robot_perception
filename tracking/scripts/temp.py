import cv2

cfg_path = "/home/asl/catkin_ws/src/indoor_mobile_robot_perception/scripts/yolov4-tiny.cfg"
weights_path = "/home/asl/catkin_ws/src/indoor_mobile_robot_perception/scripts/yolov4-tiny.weights"

net = cv2.dnn.readNet(cfg_path, weights_path)
print("YOLO model loaded successfully.")
