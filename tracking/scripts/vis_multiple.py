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

def visualization_markers(posearray,publisher):

    #Object as sphere
    # print(center)
    print('hi')
    markerarray=MarkerArray()
    tempmarker=Marker()
    
    tempmarker.header.frame_id = "map"
    tempmarker.type = Marker.SPHERE

    
    
    tempmarker.scale.x = 0.1
    tempmarker.scale.y = 0.1
    tempmarker.scale.z = 0.1

    for i in posearray.poses:
        
        tempmarker.header.stamp = rospy.Time.now()
        
        tempmarker.action = Marker.ADD
        tempmarker.id = i.ID
        tempmarker.color.r = i.ID/10
        tempmarker.color.g = 1-(i.ID/10)   #It ll saturate if no. of people being tracked is more than 10
        tempmarker.color.b = 0.5
        tempmarker.color.a = 1.0

        tempmarker.pose.position.x=i.pose.position.x
        tempmarker.pose.position.y=i.pose.position.y
        tempmarker.pose.position.z=0

        markerarray.markers.append(tempmarker)


    print("THE MARKERS:",markerarray)

    publisher.publish(markerarray)
    

def kalman_callback(pose_array):

    print("Pose array:",pose_array)
    visualization_markers(pose_array,kalman_markerarray_pub)
    kalman_pathmaker(pose_array.poses[0],kalman_path1,kalman_path_pub1)
    #kalman_pathmaker(pose_array.poses[1],kalman_path2,kalman_path_pub2)

def kalman_pathmaker(pose,kalman_path,kalman_path_pub):

    x = pose.pose.position.x
    y = pose.pose.position.y

    print("Kalman pose:",pose)

          
    kalman_path.header.frame_id="laser"
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

    print(' ')
    #visualization_markers(pose_array,measured_markerarray_pub)

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
    global kalman_path_pub1,kalman_path_pub2,measured_path_pub1,measured_path_pub2,measured_markerarray_pub,kalman_markerarray_pub
    rospy.init_node('Visualisation_Node')

    kalman_pose_sub = rospy.Subscriber('/kalmanposeArray',PoseIDArray,kalman_callback)

    measured_pose_sub = rospy.Subscriber('/Measurements',PoseIDArray,measured_callback)

    kalman_path_pub1=rospy.Publisher('/kalman_path1',Path,queue_size=20)
    kalman_path_pub2=rospy.Publisher('/kalman_path2',Path,queue_size=20)

    measured_path_pub1=rospy.Publisher('/measured_path1',Path,queue_size=20)
    measured_path_pub2=rospy.Publisher('/measured_path2',Path,queue_size=20)

    measured_markerarray_pub=rospy.Publisher('/measured_markers',MarkerArray,queue_size=20)
    kalman_markerarray_pub=rospy.Publisher('/kalman_markers',MarkerArray,queue_size=20)

    #rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()

#The nnumber of circles keeps changing