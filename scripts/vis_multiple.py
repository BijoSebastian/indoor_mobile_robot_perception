#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import math


from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,PoseStamped,PoseArray
from nav_msgs.msg import Path
from tracking.msg import PoseID,PoseIDArray

#Object Initialization
kalman_path1=Path()
kalman_path2=Path()

measured_path1=Path()
measured_path2=Path()




#Variable Initialization
actualpath = []
kalmanpath = []

#Functions

'''def shutdown():

    #Prediction for next 10 steps:
    t=0
    while(t<20):
        prediction(X,P)
        kalmanpath.append([X[0][0], X[1][0]])
        t+=1


    #pathvis(actualpath, kalmanpath)'''

'''def pathvis(path1, path2):
    print("Actualpath:")
    print(path1)
    print("Kalmanpath:")
    print(path2)
    actual_x = [coord[0] for coord in path1]
    actual_y = [coord[1] for coord in path1]
    kalman_x = [coord[0] for coord in path2]
    kalman_y = [coord[1] for coord in path2]

    plt.xlabel('x')
    plt.ylabel('y')
    plt.scatter(0,0,color="black",marker="s")
    plt.plot(actual_x, actual_y,linewidth=4,color='red')
    plt.plot(kalman_x, kalman_y,linestyle="dashed",color='blue')
    plt.title('Actual path vs Kalman path')
    plt.savefig('path2.png')
    plt.show()
    #plt.waitforbuttonpress(0)

    image = cv2.imread('path2.png')
  
    Rotated_image = imutils.rotate(image, angle=90)

    cv2.imshow("Rotated", Rotated_image)

    cv2.waitKey(0)'''

def visualization_markers(posearray):

    #Object as sphere
    # print(center)
    print('hi')
    markerarray=MarkerArray()
    tempmarker=Marker()
    
    tempmarker.header.frame_id = "map"
    tempmarker.type = Marker.SPHERE

    markerid=0
    
    tempmarker.scale.x = 0.1
    tempmarker.scale.y = 0.1
    tempmarker.scale.z = 0.1

    color=0.5

    for i in posearray.poses:
        
        tempmarker.header.stamp = rospy.Time.now()
        
        tempmarker.action = Marker.ADD
        tempmarker.id = i.ID

        tempmarker.color.r = color-0.2
        tempmarker.color.g = color
        tempmarker.color.b = color-0.3
        tempmarker.color.a = 1.0

        tempmarker.pose.position.x=i.pose.position.x
        tempmarker.pose.position.y=i.pose.position.y
        tempmarker.pose.position.z=0

        markerarray.markers.append(tempmarker)

        markerid+=1
        color+=0.2

    print("THE MARKERS:",markerarray)

    measured_markerarray_pub.publish(markerarray)
    

def kalman_callback(pose_array):

    kalman_pathmaker(pose_array.poses[0],kalman_path1,kalman_path_pub1)
    #kalman_pathmaker(pose_array.poses[1],kalman_path2,kalman_path_pub2)

def kalman_pathmaker(pose,kalman_path,kalman_path_pub):

    x = pose.position.y
    y = pose.position.x

    #print(pose)

          
    kalman_path.header.frame_id="map"
    kalman_path.header.stamp=rospy.Time.now()
    pose_current = PoseStamped()
    pose_current.pose.position.x = x
    pose_current.pose.position.y = y
    pose_current.pose.position.z = 0
    kalman_path.poses.append(pose_current)

    kalman_path_pub.publish(kalman_path)



def measured_callback(pose_array):

    # measured_pathmaker(pose_array.poses[0],measured_path1,measured_path_pub1)
    # measured_pathmaker(pose_array.poses[1],measured_path2,measured_path_pub2)

    visualization_markers(pose_array)

def measured_pathmaker(pose,measured_path,measured_path_pub):

    x = pose.position.x
    y = pose.position.y

    if(not math.isnan(x) and not math.isnan(y)):  
        measured_path.header.frame_id="map"
        measured_path.header.stamp=rospy.Time.now()
        pose_current = PoseStamped()
        pose_current.pose.position.x = x
        pose_current.pose.position.y = y
        pose_current.pose.position.z = 0
        measured_path.poses.append(pose_current)

        measured_path_pub.publish(measured_path)



def main():
    global kalman_path_pub1,kalman_path_pub2,measured_path_pub1,measured_path_pub2,measured_markerarray_pub
    rospy.init_node('Visualisation_Node')

    kalman_pose_sub = rospy.Subscriber('/kalmanposeArray',PoseArray,kalman_callback)

    measured_pose_sub = rospy.Subscriber('/PoseArray',PoseIDArray,measured_callback)

    kalman_path_pub1=rospy.Publisher('/kalman_path1',Path,queue_size=20)
    kalman_path_pub2=rospy.Publisher('/kalman_path2',Path,queue_size=20)

    measured_path_pub1=rospy.Publisher('/measured_path1',Path,queue_size=20)
    measured_path_pub2=rospy.Publisher('/measured_path2',Path,queue_size=20)

    measured_markerarray_pub=rospy.Publisher('/measured_markers',MarkerArray,queue_size=20)

    #rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()

#The nnumber of circles keeps changing