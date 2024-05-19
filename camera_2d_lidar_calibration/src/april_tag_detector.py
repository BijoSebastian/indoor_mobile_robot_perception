#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import apriltag

options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

class AprilTagPosePublisher:
    def __init__(self):
        rospy.init_node('apriltag_pose_publisher')
        self.bridge = CvBridge()
        self.tag_size = 0.163  # meters
        self.fx = 982.0966342862777  # focal length in pixels
        self.fy = 980.5978480470865
        self.pose_array = PoseArray()  # Initialize PoseArray

        # Publisher for April tag poses
        self.pose_pub = rospy.Publisher('/apriltag_poses', PoseArray, queue_size=10)

        # Publisher for April tag detection image
        self.image_pub = rospy.Publisher('/apriltag_detection_image', Image, queue_size=10)

        # Start time
        self.start_time = rospy.get_time()

        # Subscribe to image topic
        image_sub = rospy.Subscriber('usb_cam/image_raw', Image, self.image_callback)

    def run(self):
        rospy.spin()

    def image_callback(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

        self.pose_array.poses = []  # Clear the previous pose array

        k1= 0.07723030540822544 

        k2= 0.1207014523573728

        p1_k3= 0.008441174793537811

        p2_k4= 0.01473335120687576
        for r in results:
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            print("[INFO] tag ID: {}, center: ({}, {})".format(r.tag_id, cX, cY))
            

            # Draw rectangle around the April tag
            cv2.polylines(frame, [np.array(r.corners, dtype=np.int32)], True, (0, 255, 0), 2)

            # Draw center of the April tag
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            tag_corners = np.array(r.corners, dtype=np.float32)

            perceived_width = np.linalg.norm(tag_corners[0] - tag_corners[1])



            #camera calibration matrix and distortion coefs (zeros)

            camMtrx = np.array([[943.6798948965514, 0, 357.1507412220593],

                            [ 0,  942.579608008557, 268.3416291872803],

                            [ 0,          0,            1]])

            distortCoefs = np.array([k1, k2, p1_k3, p2_k4])



            #define tag dimensions in inches, centered on zero

            tagSizePnts = np.array([(-6.417323,-6.417323,0), (-6.417323,6.417323,0), (6.417323,6.417323,0), (6.417323, -6.417323, 0)])



            fndPose, vRot, vTran = cv2.solvePnP(tagSizePnts, r.corners, camMtrx, distortCoefs, flags=0)

            #distance = self.fx * self.tag_size / perceived_width

            distance = vTran[2]

            dX = vTran[0]

            dY = vTran[1]

            print('Dx,Dy,dist:',dX,dY,distance)   
            # Calculate distance
            # tag_corners = np.array(r.corners, dtype=np.float32)
            # perceived_width = np.linalg.norm(tag_corners[0] - tag_corners[1])
            # distance = self.fx * self.tag_size / perceived_width

            # dX = ((cX - 240) * distance) / self.fx
            # dY = ((cY - 320) * distance) / self.fy

            pose = Pose()
            pose.position.x = dX
            pose.position.y = distance
            self.pose_array.poses.append(pose)

            # Print distance
            print("Tag ID: %d, Distance: %.2f m" % (r.tag_id, distance))
            print("center: ({}, {})".format(cX, cY))

        # Publish the image with April tag detections
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

        # Set the header of the pose array
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_array.header.frame_id = 'base_frame'

        # Publish the updated pose array
        self.pose_pub.publish(self.pose_array)

    def cleanup(self):
        rospy.loginfo("[INFO] Cleaning up...")

if __name__ == '__main__':
    try:
        apriltag_pose_publisher = AprilTagPosePublisher()
        apriltag_pose_publisher.run()
        
    except rospy.ROSInterruptException:
        pass
