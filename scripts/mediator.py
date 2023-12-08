#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2  # importing cv

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray
from nav_msgs.msg import Path


cam_poses=[]

def cam_callback(msg):

    for i in msg.poses:
        cam_pose=[(i.position.x),(i.position.y)]
        cam_poses.append(cam_pose)
    
    print(cam_poses)

def lidar_callback(msg):
    lidar_poses=[]

    for i in msg.lidar_poses:
        lidar_pose=[(i.position.x),(i.position.y)]
        lidar_poses.append(lidar_pose)

    #convert to cam pose
    
    print(lidar_poses)



def main():
    global camposepub
    rospy.init_node('MediatorNode')

    cam_pose_sub = rospy.Subscriber('/CamDetections', PoseArray, cam_callback)
    lidar_pose_sub = rospy.Subscriber('/LidarDetections', PoseArray, lidar_callback)


    rospy.spin()


if __name__ == '__main__':
    main()


