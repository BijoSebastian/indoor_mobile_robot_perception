#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
#from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

from scipy.optimize import linear_sum_assignment
from tracking.msg import PoseID,PoseIDArray


predictions=[]


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


def callback_filtered_laser(poses):

    lidar_measurements=[]
    selected_lidar=[]
    FinalMeasurements=PoseIDArray()
    for i in poses.poses:
        lidar_measurements.append([i.position.x,i.position.y])

    
    cost=cost_matrix(lidar_measurements,predictions)

    #Solve the assignment problem
    row_indices, col_indices = linear_sum_assignment(cost)

    # # Extract the optimal assignment
    assignment = [(row, col) for row, col in zip(row_indices, col_indices)]


    #Handle case when we get extra measurements

    print("Optimal Assignment:")
    #print(len(cam_poses))
    #print(len(lidar_poses))
    for row, col in assignment:
        print(f"Pose {lidar_measurements[row]} in poses1, assigned to poses {predictions[col]} in Poses2")
        
        if(np.linalg.norm(np.array(lidar_measurements[row])-np.array(predictions[col]))<=2):
            selected_lidar.append(lidar_measurements[row])

            FinalMeasurement=PoseID()
            FinalMeasurement.pose.position.x=lidar_measurements[row][0]
            FinalMeasurement.pose.position.y=lidar_measurements[row][1]
            FinalMeasurement.ID=ids[col]

            FinalMeasurements.poses.append(FinalMeasurement)
        

    for k in selected_lidar:
        if(k in lidar_measurements):
            lidar_measurements.remove(k)
            print("Got removed:",k)

    for j in lidar_measurements:
        FinalMeasurement=PoseID()
        FinalMeasurement.pose.position.x=j[0]
        FinalMeasurement.pose.position.y=j[1]
        FinalMeasurement.ID=0

        FinalMeasurements.poses.append(FinalMeasurement)

    measurepub.publish(FinalMeasurements)


    

    

def callback_predicted(poses):

    global predictions,ids
    predictions=[]
    ids=[]

    for i in poses.poses:
        predictions.append([i.pose.position.x,i.pose.position.y])
        ids.append(i.ID)
    

    
    
def main():

    global measurepub,visualpub
    rospy.init_node('Hungarian_Algorithm')
    

    pose_filtered_sub=rospy.Subscriber('/PoseFilteredLaser',PoseArray,callback_filtered_laser)

    pose_predictions_sub=rospy.Subscriber('/PredictedPoses',PoseIDArray,callback_predicted)

    measurepub=rospy.Publisher('/Measurements',PoseIDArray,queue_size=10)

    # visualpub=rospy.Publisher('/Pose',Pose,queue_size=10)

    
    rospy.spin()
        
  

if __name__ == '__main__':
    main()
    
