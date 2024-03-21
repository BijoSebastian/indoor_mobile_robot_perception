#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
#from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import message_filters
from std_msgs.msg import Header

from scipy.optimize import linear_sum_assignment
from tracking.msg import PoseID,PoseIDArray


#predictions=[]


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


def callback_filtered_laser(filter_poses):

    print("Filtered poses time:")
    
    print(filter_poses.header.stamp)

    # lidar_measurements=[]
    # selected_lidar=[]
    # final_ptime=filter_poses.header.stamp
    # FinalMeasurements=PoseIDArray()
    # FinalMeasurements.header= Header(stamp=final_ptime,frame_id='base_frame')
    # for i in filter_poses.poses:
    #     lidar_measurements.append([i.position.x,i.position.y])

    

    # for j in lidar_measurements:
    #     FinalMeasurement=PoseID()
    #     FinalMeasurement.pose.position.x=j[0]
    #     FinalMeasurement.pose.position.y=j[1]
    #     FinalMeasurement.ID=0

    #     FinalMeasurements.poses.append(FinalMeasurement)

    # measurepub.publish(FinalMeasurements)
    # firsttime=False


    

    

def callback_predicted(poses):

    print("Predicted poses time:")
    print(poses.header.stamp)

    # global predictions,ids
    # predictions=[]
    # ids=[]

    # for i in poses.poses:
    #     predictions.append([i.pose.position.x,i.pose.position.y])
    #     ids.append(i.ID)
    

def callback(filter_poses,predicted_poses):

    print("Entered callback")

    #Predicted poses part
    predictions=[]
    ids=[]

    for i in predicted_poses.poses:
        predictions.append([i.pose.position.x,i.pose.position.y])
        ids.append(i.ID)

    #Filtered poses part
        
    
    lidar_measurements=[]
    selected_lidar=[]
    final_ptime=filter_poses.header.stamp
    FinalMeasurements=PoseIDArray()
    FinalMeasurements.header= Header(stamp=final_ptime,frame_id='base_frame')
    for i in filter_poses.poses:
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

    print(predictions)
    if(not (predictions[0][0]==0 and predictions[0][1]==0)):
        for j in lidar_measurements:
            FinalMeasurement=PoseID()
            FinalMeasurement.pose.position.x=j[0]
            FinalMeasurement.pose.position.y=j[1]
            FinalMeasurement.ID=0

            FinalMeasurements.poses.append(FinalMeasurement)
    else:
        print('Simply dummies')
        #Popping those which got appended in optimal assignment
        for i in FinalMeasurements.poses:
            FinalMeasurements.poses.pop()
        print(FinalMeasurements.poses)

    measurepub.publish(FinalMeasurements)
        
    
def main():

    global measurepub,visualpub,firsttime
    rospy.init_node('Hungarian_Algorithm')
    firsttime=True
    

    # pose_filtered_sub=rospy.Subscriber('/PoseFilteredLaser',PoseArray,callback_filtered_laser)

    # pose_predictions_sub=rospy.Subscriber('/PredictedPoses',PoseIDArray,callback_predicted)

    pose_filtered_sub = message_filters.Subscriber('/PoseFilteredLaser', PoseArray, queue_size=10)
    pose_predictions_sub = message_filters.Subscriber('/PredictedPoses', PoseIDArray, queue_size=10)

    #Debug this by checking if its actaully the problem with time sychronisaer. echo the above topics

    ts = message_filters.ApproximateTimeSynchronizer([pose_filtered_sub,pose_predictions_sub], 10, 13) #try varying time diff


    ts.registerCallback(callback)


    measurepub=rospy.Publisher('/Measurements',PoseIDArray,queue_size=10)

    # visualpub=rospy.Publisher('/Pose',Pose,queue_size=10)

    
    rospy.spin()
        
  

if __name__ == '__main__':
    main()
    
