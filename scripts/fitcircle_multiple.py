#!/usr/bin/env python3

import rospy
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import time
import cv2 


from sensor_msgs.msg import LaserScan
import math
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseArray

#Maybe later we could use Pose stamped


# Global Variables
marker = Marker()
scan = LaserScan()
pose_array=PoseArray()

# Parameters
window_size = 10  # Sliding window size
person_threshold = 0.05  # Threshold to classify as a person





def fit_circle(points):
    if len(points) < 3:
        return None  # Insufficient points to fit a circle

    data = np.array(points, dtype=np.float32)
    circle = cv2.minEnclosingCircle(data)
    
    return circle

def polar_to_rectangular(Allclusters):
    rectangular_clusters = []
    
    for cluster in Allclusters:
        rectangular_points = []
        
        for point in cluster.reshape(-1,2):
            range_val, angle_val = point[0], point[1]
            x = range_val * np.cos(angle_val)
            y = range_val * np.sin(angle_val)
            rectangular_points.append([x, y])
        
        rectangular_clusters.append(rectangular_points)
    
    return rectangular_clusters

def polartorect(randtheta):
    return [randtheta[0] * math.cos(randtheta[1]), randtheta[0] * math.sin(randtheta[1])]

def polardistance(a, b):
    return math.sqrt(a[0]**2 + b[0]**2 - 2 * a[0] * b[0] * math.cos(a[1] - b[1]))


def visualization_point(center):

    #Object as sphere
    print(center)
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = 1

    marker.color.r = 0.5
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.pose.position.x=center[0]
    marker.pose.position.y=center[1]
    marker.pose.position.z=0

    print("THE MARKER:",marker)

    visualpub.publish(marker)



    

def callback(msg):
    # Getting the polar coordinates of the point cloud
    global pose

    #Local variables
    # Initialize a list to store (x, y) points for circle fitting
    window_points = []
    # Initialize lists to store circle data
    circle_centers = []
    circle_radii = []

    
    pts_r = np.array(msg.ranges)
    delfi = msg.angle_increment
    pts_ang = np.arange(start=msg.angle_min, stop=msg.angle_max, step=delfi)

    pts_r_list = list(pts_r)
    pts_ang_list = list(pts_ang)

    newscan_rect = []  #Contains point cloud in rectangular coordinates
    newscan_polar=[] #Contains point cloud in polar coordinates
    for r, ang in zip(pts_r_list, pts_ang_list):
        if (not np.isinf(r) and abs(r)<1):#Constraining scan to 1 radius circle 'and abs(r)<1'
            newscan_polar.append([r,ang])
            newscan_rect.append(polartorect([r, ang]))

    #We now have the scan in 1m radius. In polar as well as in rect.

    try:

        for x,y in newscan_rect:

            window_points.append((x, y))
            # totr=0

            # for i in window_points:
            #     r=math.sqrt((i[0]**2) + (i[1]**2))
            #     totr+=r

            # avgr=totr/len(window_points)
            # window_size=3.33/avgr
                

            # Check if the window size is reached
            if len(window_points) >= window_size:
            # Fit a circle to the points in the sliding window
                circle = fit_circle(window_points)

            # Classify as person or wall based on the radius
                if circle is not None:
                    center, radius = circle
                    if radius < person_threshold:
                        print(f"Person detected at ({center[0]}, {center[1]}), Radius: {radius}")
                    else:
                        print(f"Wall detected at ({center[0]}, {center[1]}), Radius: {radius}")
                
                    # Store circle data for visualization
                    circle_centers.append(center)
                    circle_radii.append(radius)

                    



            # Clear the window_points list for the next window
                window_points.clear()

        print("The circle centers are:")
        print(circle_centers)
        # center_x=circle_centers[0][0]
        # center_y=circle_centers[0][1]
        
        posetemp=Pose()
        pose_array_temp=[]

        for i in circle_centers:
            posetemp.position.x=i[0]
            posetemp.position.y=i[1]

            pose_array_temp.append(posetemp)

        pose_array.poses=pose_array_temp

        #visualization_point(circle_centers[3])

        posepub.publish(pose_array)

        


    except:

        pose_array.poses=[[]]
        # pose.position.x=math.nan
        # pose.position.y=math.nan

        print(pose_array.poses)

        posepub.publish(pose_array)


def main():

    global posepub,visualpub
    rospy.init_node('fit_circle')
    
    sub = rospy.Subscriber('/scan', LaserScan, callback)

    posepub=rospy.Publisher('/PoseArray',PoseArray,queue_size=10)

    visualpub=rospy.Publisher('/visualpose3',Marker,queue_size=10)


if __name__ == '__main__':
    main()
    rospy.spin()






