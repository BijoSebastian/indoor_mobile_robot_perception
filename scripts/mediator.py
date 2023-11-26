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




def callback(msg):
    poses=[]

    for i in msg.poses:
        pose=[(i.position.x-320),-(i.position.y-240)]
        poses.append(pose)
    
    print(poses)

def main():
    global camposepub
    rospy.init_node('MediatorNode')
    pose_sub = rospy.Subscriber('/detection_topic', PoseArray, callback)

    #camposepub=rospy.Publisher('/camposeArray',PoseArray,queue_size=20)

    rospy.spin()


if __name__ == '__main__':
    main()


