#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose,PoseArray
import numpy as np

from scipy.optimize import linear_sum_assignment

import math

#Functions

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

# pose1=[[1,1],
#        [5,5],
#        [3,3]]

# pose2=[[1,0],
#        [2,0]]

# cost=cost_matrix(pose1,pose2)



def callback_lidar(msg):

    global lidar_poses
    lidar_poses=[]
    for i in msg.poses:
        print(i)
        lidar_poses.append([i.position.x,i.position.y])

    

def callback_cam(msg):

    global cam_poses
    cam_poses=[]
    for i in msg.poses:
        cam_poses.append([i.position.x,i.position.y])

    cost=cost_matrix(lidar_poses,cam_poses)

    #Solve the assignment problem
    row_indices, col_indices = linear_sum_assignment(cost)

    # # Extract the optimal assignment
    assignment = [(row, col) for row, col in zip(row_indices, col_indices)]

    print("Optimal Assignment:")
    for row, col in assignment:
        print(f"Pose {lidar_poses[row]} in poses1, assigned to poses {cam_poses[col]} in Poses2")

    




def main():

    global posepub,visualpub
    rospy.init_node('Hungarian_Algorithm')
    

    pose_lidar_sub=rospy.Subscriber('/PoseLidar',PoseArray,callback_lidar)

    pose_cam_sub=rospy.Subscriber('/PoseCam',PoseArray,callback_cam)

    visualpub=rospy.Publisher('/Pose',Pose,queue_size=10)

    '''while(not rospy.is_shutdown()):
        
        posepub.publish(pose)
        #visualpub.publish(marker)'''
    
    rospy.spin()
        
  

if __name__ == '__main__':
    main()
    

# '''Kalman filter to find prediciton of bounding positions after t+1. Then perform linear sum assignment'''



