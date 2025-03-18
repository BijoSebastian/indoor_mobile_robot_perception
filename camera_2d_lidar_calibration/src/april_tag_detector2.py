#!/usr/bin/env python3

import rospy
import cv2
import apriltag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import tf.transformations
from tracking.msg import PoseID, PoseIDArray
import csv
import os
import rospkg

class AprilTagDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        self.image_sub = rospy.Subscriber('/usb_cam/image_rect', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/camera/image_annotated', Image, queue_size=10)
        self.lidar_pose_pub = rospy.Publisher('/lidar_apriltag_pose', PoseIDArray, queue_size=10)

        # Camera intrinsic parameters
        self.camera_matrix = np.array([
            [953.5454154705232, 0, 301.5036500757761],
            [0, 962.3682469119241, 163.5662975387139],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([0.06981473831094037, 0.01023196516465969, -0.02452232979891343, -0.00912126640237256, 0])

        # Define the tag size in meters
        self.tag_size = 0.16

        # Load the extrinsic parameters
        #self.load_extrinsic_parameters()
        qx = 0.9861527612744059

        qy = 0.0076052073314031125

        qz = -0.07028492197118058

        qw = 0.15001640575555797

        tx = 0.0010316738086038613

        ty = -0.26032098490345245-0.08
        

        tz = -0.006141912038532205



        q = tf.transformations.quaternion_matrix([qw, qx, qy, qz])

        #Manually adding translation
        tx=0
        ty=-0.05
        tz=0

        q[0, 3] = tx

        q[1, 3] = ty    

        q[2, 3] = tz

        



        rospy.loginfo("Extrinsic parameter - camera to LiDAR:")

        rospy.loginfo(q)

        self.tvec_c_to_l = q[:3, 3]

        self.rot_mat_c_to_l = q[:3, :3]
        


        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('camera_2d_lidar_calibration')  # Package name
        self.csv_file = os.path.join(pkg_path, 'src', 'pose_data.csv')


        # CSV file initialization
        #self.csv_file = "/home/winston/catkin_ws/src/indoor_mobile_robot_perception/camera_2d_lidar_calibration/src/pose_data.csv"  # Specify your desired file path here
        with open(self.csv_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Tag ID", "X", "Y", "Z"])




    def load_extrinsic_parameters(self):
        rospy.loginfo("Loading extrinsic parameters from file")
        calib_file = rospy.get_param("~calib_file")
        
        with open(calib_file, 'r') as f:
            data = f.read().split()
            qx = float(data[0])
            qy = float(data[1])
            qz = float(data[2])
            qw = float(data[3])
            tx = float(data[4])
            ty = float(data[5])
            tz = float(data[6])
        
        q = tf.transformations.quaternion_matrix([qw, qx, qy, qz])
        #Manually adding translation
        tx=0
        ty=-0.02
        tz=-0.06

        q[0, 3] = tx
        q[1, 3] = ty
        q[2, 3] = tz

        rospy.loginfo("Extrinsic parameter - camera to LiDAR:")
        rospy.loginfo(q)

        self.tvec_c_to_l = q[:3, 3]
        self.rot_mat_c_to_l = q[:3, :3]

    def image_callback(self, msg):
        global present_timestamp
        present_timestamp = msg.header.stamp
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert the image to grayscale for more efficient and effective AprilTag detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)

        for tag in tags:
            pose = self.estimate_pose(tag)
            if pose is not None:
                self.draw_tag(cv_image, tag)
                print('pose in camera frame:',pose)
                self.write_pose_to_csv(pose)
                lidar_pose = self.transform_to_lidar_frame_manual(pose)
                print('pose in lidar frame:',lidar_pose)
                self.publish_lidar_pose(lidar_pose, present_timestamp)
                #self.publish_lidar_pose(pose, present_timestamp)

        try:
            annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(annotated_image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def estimate_pose(self, tag):
        object_points = np.array([
            [-0.5, -0.5, 0],
            [0.5, -0.5, 0],
            [0.5, 0.5, 0],
            [-0.5, 0.5, 0]
        ]) * self.tag_size

        image_points = np.array([
            [tag.corners[0][0], tag.corners[0][1]],
            [tag.corners[1][0], tag.corners[1][1]],
            [tag.corners[2][0], tag.corners[2][1]],
            [tag.corners[3][0], tag.corners[3][1]]
        ])

        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
        if success:
            pose = PoseID()
            pose.ID=tag.tag_id
            pose.pose.position.x = tvec[0][0]
            pose.pose.position.y = tvec[1][0]
            pose.pose.position.z = tvec[2][0]

            # Convert rotation vector to quaternion
            # rot_matrix = cv2.Rodrigues(rvec)[0]
            # quat = tf.transformations.quaternion_from_matrix(np.hstack((rot_matrix, [[0], [0], [0]])))
            # pose.orientation.x = quat[0]
            # pose.orientation.y = quat[1]
            # pose.orientation.z = quat[2]
            # pose.orientation.w = quat[3]

            return pose
        else:
            return None

    # def transform_to_lidar_frame(self, pose):
    #     #position_camera = np.array([pose.position.x, pose.position.y, pose.position.z])
    #     position_camera = np.array([pose.position.x, -pose.position.y, -pose.position.z])
    #     position_lidar = self.rot_mat_c_to_l.dot(position_camera) + self.tvec_c_to_l

    #     lidar_pose = Pose()
    #     lidar_pose.position.x = position_lidar[0]
    #     lidar_pose.position.y = position_lidar[1]
    #     lidar_pose.position.z = position_lidar[2]

    #     return lidar_pose
    
    def transform_to_lidar_frame_manual(self, pose):
        #position_camera = np.array([pose.position.x, pose.position.y, pose.position.z])
        position_camera = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        position_lidar = position_camera - self.tvec_c_to_l
        lidar_pose = PoseID()
        lidar_pose.ID=pose.ID
        lidar_pose.pose.position.x = position_lidar[0]
        lidar_pose.pose.position.y = position_lidar[1]
        lidar_pose.pose.position.z = position_lidar[2]

        return lidar_pose

    def publish_lidar_pose(self, pose, timestamp):
        pose_array = PoseIDArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = "lidar_link"
        pose_array.poses.append(pose)
        self.lidar_pose_pub.publish(pose_array)

    def draw_tag(self, image, tag):
        # Draw the bounding box
        for idx in range(len(tag.corners)):
            cv2.line(image,
                     tuple(tag.corners[idx-1].astype(int)),
                     tuple(tag.corners[idx].astype(int)),
                     (0, 255, 0), 2)

        # Draw the center of the tag
        cv2.circle(image, tuple(tag.center.astype(int)), 5, (0, 0, 255), -1)

        # Draw the tag ID
        cv2.putText(image, str(tag.tag_id), 
                    tuple(tag.center.astype(int)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (255, 0, 0), 2)
    
    def write_pose_to_csv(self, pose):
        with open(self.csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            print('WRITING')
            writer.writerow([pose.ID, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])

if __name__ == '__main__':
    rospy.init_node('april_tag_detector', anonymous=True)

    detector = AprilTagDetector()
    rospy.spin()
