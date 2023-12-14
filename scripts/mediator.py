#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2  # importing cv

from scipy.optimize import linear_sum_assignment

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray
from nav_msgs.msg import Path
from tracking.msg import PoseIDArray,PoseID

#Initialization
predicted_poses=[]
ids=[]

def cost_matrix(poses1,poses2):

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



def finalposes_callback(msg):
    global final_poses
    final_poses=[]
    Measurements=PoseIDArray()
    

    for i in msg.poses:
        final_pose=[(i.position.x),(i.position.z)]
        final_poses.append(final_pose)
        
    
    #print(final_poses)

    #Hungarian

    cost=cost_matrix(final_poses,predicted_poses)

    #Solve the assignment problem
    row_indices, col_indices = linear_sum_assignment(cost)

    # # Extract the optimal assignment
    assignment = [(row, col) for row, col in zip(row_indices, col_indices)]

    print("Optimal Assignment:")
    #print(len(cam_poses))
    #print(len(lidar_poses))
    print(assignment)
    for row, col in assignment:
        print(f"Pose {final_poses[row]} in poses1, assigned to poses {predicted_poses[col]} in Poses2")

        Measure_id=PoseID()
        Measure_id.ID=ids[col]
        
        measurementpose=final_poses[row]
        Measure_id.pose.position.x=measurementpose[0]
        Measure_id.pose.position.z=measurementpose[1]
        Measurements.poses.append(Measure_id)
    
    for k in range(0,len(final_poses)):
        Measure_id=PoseID()
        assignment=[[1,1]]
        ass=np.array(assignment)
        if(k not in ass[:,0]):
            measurementpose=final_poses[k]
            
            Measure_id.ID=0
            Measure_id.pose.position.x=measurementpose[0]
            Measure_id.pose.position.z=measurementpose[1]
            Measurements.poses.append(Measure_id)

        
    #for those that didnt get matched to any predictedpose, keep id as math.nan and publish them also.
    posepub.publish(Measurements)


def predictedposes_callback(msg):
    global predicted_poses,ids
    predicted_poses=[]
    ids=[]
    

    for i in msg.poses:
        predicted_pose=[(i.pose.position.x),(i.pose.position.y)]
        id=i.ID
        predicted_poses.append(predicted_pose)
        ids.append(id)
    
    print(predicted_poses)
    print(ids)

    

def main():
    global posepub
    rospy.init_node('MediatorNode')

    final_poses_sub = rospy.Subscriber('/FinalPoses', PoseArray, finalposes_callback)
    predicted_poses_sub = rospy.Subscriber('/PredictedPoses', PoseIDArray, predictedposes_callback)

    posepub=rospy.Publisher('/PoseArray',PoseIDArray,queue_size=10)


    rospy.spin()


if __name__ == '__main__':
    main()

#check if this works