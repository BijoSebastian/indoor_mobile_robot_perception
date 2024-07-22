#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
import message_filters
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from geometry_msgs.msg import PoseArray,Pose
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
from scipy.optimize import linear_sum_assignment
from std_msgs.msg import Header
import time

def ensure_2d_array(arr):
  """Converts an array to a 2D NumPy array.

  Args:
      arr: The input array.

  Returns:
      A 2D NumPy array, even if the input was a 1D array.
  """
  return np.atleast_2d(arr)


#HUNGARIAN ALGORITHM
def cost_matrix(poses1,poses2):

    poses1=ensure_2d_array(poses1)
    poses2=ensure_2d_array(poses2)

    cost_mat=np.empty([len(poses1),len(poses2)])
    row=0
    col=0
    poses1=np.array(poses1)
    poses2=np.array(poses2)
    
    for i in poses1:
        for j in poses2:
            dist=np.linalg.norm((i-j))
            cost_mat[row,col]=dist
            col+=1
        col=0
        row+=1
    
    return cost_mat

#Functions for object detecion
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

    posearray = PoseArray()

    for i in range(len(boxes)):        

        if i in indexes:

            x, y, w, h = boxes[i]

            pose = Pose()

            label = str(classes[class_ids[i]])

            if class_ids[i] == 0:

                centers.append([x+w//2, y+h//2])

                center_x= x+w//2

                center_y= y+h//2

                pose.position.x= center_x#-320#Tranforming frame to centre of image

                pose.position.y= center_y#-240)

                pose.position.z= 0

                posearray.poses.append(pose)
                cam_detections.append([center_x,center_y])
            
    for i in range(len(posearray.poses)):
        try:
            cv2.circle(img, (int(round(posearray.poses[i].position.x)),int(round(posearray.poses[i].position.y))), detection_point_radius, (255,0,0), -1)
        except :
            continue

    return posearray,cam_detections

#Function for reprojection
def get_z(T_cam_world, T_world_pc, K):
    R = T_cam_world[:3,:3]
    t = T_cam_world[:3,3]
    proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
    if len(T_world_pc.shape) == 1:
        T_world_pc = np.atleast_2d(T_world_pc)

    xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
    xy_hom = np.dot(proj_mat, xyz_hom.T).T
    z = xy_hom[:, -1]
    z = np.asarray(z).squeeze()
    return z

def extract(point):
    return [point[0], point[1], point[2]]

def extract_PoseArray(point):
    x=[point.position.x,point.position.y,0]
    return x
#Callback functions
def callback(image,lidar_detections,scan):
    print('Callback started!')

    before_callback_time=rospy.Time.now()
    before_callback_time_sec=before_callback_time.to_nsec()*(10**(-9))
    
    img = bridge.imgmsg_to_cv2(image)
    img=np.array(img)
    h=image.height
    w=image.width
    img_blank=np.zeros((h,w,3))
    img=np.vstack((img,img_blank))
    img = np.uint8(img)

    #Scan part
    #The scan is projected on the image as green circles
    cloud = lp.projectLaser(scan)
    points = pc2.read_points(cloud)
    objPoints = np.array([extract(point) for point in points])
    
    Z = get_z(q, objPoints, K)
    
    objPoints = objPoints[Z > 0]
    
    if lens == 'pinhole':
        img_points, _ = cv2.projectPoints(objPoints, rvec, tvec, K, D)
        
    elif lens == 'fisheye':
        objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
        img_points, _ = cv2.fisheye.projectPoints(objPoints, rvec, tvec, K, D)
        
    
    img_points = np.squeeze(img_points)

    for i in range(len(img_points)):
        try:
            cv2.circle(img, (int(round(img_points[i][0])),int(round(img_points[i][1]))), laser_point_radius, (0,255,0), 1)
        except Exception as err:
            print('!!!!Error:',err)
            continue
    
    #Image part
    
    camposearray,cam_detections=Object_detection_yolo(img) #The positions of people on image is returned as PoseArray and list
    camposearray.header.stamp=image.header.stamp #Camera Detections have same timestamp as the image itself

    #Detection part

    lidar_detection_ptime=lidar_detections.header.stamp

    paired_pose_array=[]
    filtered_pose_array=PoseArray()
    filtered_pose_array.header= Header(stamp=lidar_detection_ptime,frame_id='base_frame') #Filtered poses time will be the detections time. And detections time is the scans time.s

    obj_detected_Points = np.array([extract_PoseArray(point) for point in lidar_detections.poses])
    if(len(obj_detected_Points)!=0):
        Z_detections = get_z(q, obj_detected_Points, K)
        obj_detected_Points=obj_detected_Points[Z_detections>0]

        if ((not np.any(obj_detected_Points)) or (not np.any(cam_detections))):
            print("!!!!No detected points from LiDAR or Camera. Skipping projection of detected points.!!!!")

        else:
        
            #try:
            if lens == 'pinhole':
                #Detections Projection
                detected_points, _ = cv2.projectPoints(obj_detected_Points, rvec, tvec, K, D)
            elif lens == 'fisheye':
                #Detections Projection
                obj_detected_Points = np.asarray(obj_detected_Points)  # Ensure NumPy array
                # Check if single element or 1D array and reshape if necessary
                #if len(obj_detected_Points.shape) == 0 or obj_detected_Points.shape[0] == 1:
                obj_detected_Points = np.atleast_2d(obj_detected_Points)

                if(np.shape(obj_detected_Points)==(1,1,3)):
                    obj_detected_Points=obj_detected_Points[0]
                obj_detected_Points = np.reshape(obj_detected_Points, (1,obj_detected_Points.shape[0],obj_detected_Points.shape[1]))
                detected_points, _ = cv2.fisheye.projectPoints(obj_detected_Points, rvec, tvec, K, D)
            
            detected_points = np.round(np.squeeze(detected_points))
            detected_points=detected_points.astype(int)
            for i in range(len(detected_points)):
                    if not np.isscalar(detected_points[i]):  # Check if not a scalar
                        if len(detected_points[i]) == 2:
                            cv2.circle(img, (int(round(detected_points[i][0])),int(round(detected_points[i][1]))), detection_point_radius, (0,0,255), -1)
                        else:
                            print(f"!!!!Error: detected_points[{i}] is not a valid 2D tuple. Skipping circle.!!!!")
                    else:
                        print(f"!!!!Error: detected_points[{i}] is a scalar after projection. Skipping circle.!!!!")


                    

            cost=cost_matrix(detected_points,cam_detections)
            

            #Solve the assignment problem
            row_indices, col_indices = linear_sum_assignment(cost)

            # # Extract the optimal assignment
            assignment = [(row, col) for row, col in zip(row_indices, col_indices)]
            
            for row, col in assignment:
                # print('row, col:',row, col)
                # print('obj_detected_Points:',obj_detected_Points)
                # print(f"Pose {obj_detected_Points[0][row]} in poses1, assigned to poses {cam_detections[col]} in Poses2")
                # print('detected_points:',detected_points)
                # print('detected_point:',detected_points[row])
                # print('cam_detections:',cam_detections[col])
                # print('Type:',type(detected_points[row]))
                if isinstance(detected_points[row], (list, np.ndarray)):
                    cv2.line(img, detected_points[row], cam_detections[col], (255, 255, 0) , 5) 
                else:
                    cv2.line(img, detected_points, cam_detections[col], (255, 255, 0) , 5)

                paired_pose=[obj_detected_Points[0][row],cam_detections[col]] #[Laser detections, camera detections]
                paired_pose_array.append(paired_pose)

            for k in paired_pose_array:
                filtered_pose=Pose()
                filtered_pose.position.x,filtered_pose.position.y=k[0][0],k[0][1]
                filtered_pose_array.poses.append(filtered_pose)

    else:
        print('!!!!Obj_detected_Points is an empty list!!!!')        
        
    after_callback_time=rospy.Time.now()
    after_callback_time_sec=after_callback_time.to_nsec()*(10**(-9))
    time_elapse=after_callback_time_sec-before_callback_time_sec

    filtered_laser_pub.publish(filtered_pose_array)
    pub.publish(bridge.cv2_to_imgmsg(img))
    detection_pub.publish(camposearray)

        

rospy.init_node('reprojection')
scan_topic = rospy.get_param("~scan_topic")
image_topic = rospy.get_param("~image_topic")
detection_topic = rospy.get_param("~detected_topic") #ADD THIS IN LAUNCH FILE
calib_file = rospy.get_param("~calib_file")
config_file = rospy.get_param("~config_file")
laser_point_radius = rospy.get_param("~laser_point_radius")
detection_point_radius = rospy.get_param("~detection_point_radius")
time_diff = rospy.get_param("~time_diff")
bridge = CvBridge()
lp = lg.LaserProjection()

print("LOADING YOLO")

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


with open(calib_file, 'r') as f:
    data = f.read().split()
    qx = float(data[0])
    qy = float(data[1])
    qz = float(data[2])
    qw = float(data[3])
    tx = float(data[4])
    ty = float(data[5])
    tz = float(data[6])
q = Quaternion(qw,qx,qy,qz).transformation_matrix
q[0,3] = tx
q[1,3] = ty
q[2,3] = tz
print("Extrinsic parameter - camera to laser")
print(q)
tvec = q[:3,3]
rot_mat = q[:3,:3]
rvec, _ = cv2.Rodrigues(rot_mat)

with open(config_file, 'r') as f:
    f.readline()
    config = yaml.load(f)
    lens = config['lens']
    fx = float(config['fx'])
    fy = float(config['fy'])
    cx = float(config['cx'])
    cy = float(config['cy'])
    k1 = float(config['k1'])
    k2 = float(config['k2'])
    p1 = float(config['p1/k3'])
    p2 = float(config['p2/k4'])  
K = np.matrix([[fx, 0.0, cx],
               [0.0, fy, cy],
               [0.0, 0.0, 1.0]])
D = np.array([k1, k2, p1, p2])
print("Camera parameters")
print("Lens = %s" % lens)
print("K =")
print(K)
print("D =")
print(D)

pub = rospy.Publisher("/reprojection", Image, queue_size=10)
detection_pub = rospy.Publisher('/PoseCamera', PoseArray, queue_size=10) #/CameraPoses
filtered_laser_pub = rospy.Publisher('/PoseFilteredLaser', PoseArray, queue_size=10) 

scan_sub = message_filters.Subscriber(scan_topic, LaserScan, queue_size=10)
image_sub = message_filters.Subscriber(image_topic, Image, queue_size=10)
detection_sub = message_filters.Subscriber(detection_topic, PoseArray, queue_size=10) #Added a new subscriber to /LidarPoses Topic
ts = message_filters.ApproximateTimeSynchronizer([image_sub,detection_sub,scan_sub], 10, time_diff) #
ts.registerCallback(callback)
rospy.spin()


