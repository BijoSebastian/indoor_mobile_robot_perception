#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize CvBridge
bridge = CvBridge()

# Callback function
def callback(image):

    # Convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")  # Convert to BGR format for OpenCV

    rotated_image = cv2.rotate(cv_image, cv2.ROTATE_180)
    mirrored_image = cv2.flip(rotated_image, 1)
    # Display the image using cv2.imshow()
    # cv2.imshow("CoppeliaSim Camera Image", mirrored_image)
    # cv2.waitKey(1)  # Wait for a short duration to allow OpenCV to display the image

    # Copy the timestamp from the input image message
    timestamp = image.header.stamp

    # Convert the mirrored image back to a ROS Image message
    ros_image = bridge.cv2_to_imgmsg(mirrored_image, encoding="bgr8")  # Convert to BGR format for ROS

    # Set the timestamp of the ROS Image message
    ros_image.header.stamp = timestamp

    pub.publish(ros_image)

def main():
    global pub
    rospy.init_node('Camera_Data')

    # Initialize ROS publisher and subscriber
    pub = rospy.Publisher("usb_cam/image_rect_color", Image, queue_size=10)
    image_sub = rospy.Subscriber('/coppeliasim_cam/image_raw', Image, callback)

    rospy.spin()

if __name__ == '__main__':
    main()
