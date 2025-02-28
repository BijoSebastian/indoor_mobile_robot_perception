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


def callback_filtered_laser(filtered_pose_array):

    global firsttime
    print('Scan callback entered.')
    if(not firsttime):
        ided_pose_array=PoseIDArray()
        ided_pose_array.header.stamp=filtered_pose_array.header.stamp
        print('First Filter Pose Time:',(filtered_pose_array.header.stamp.to_nsec())*(10**(-9)))
        for j in filtered_pose_array.poses:
            ided_pose=PoseID()
            ided_pose.pose.position.x=j.position.x
            ided_pose.pose.position.y=j.position.y
            ided_pose.ID=0

            ided_pose_array.poses.append(ided_pose)
        firsttime= True
        measurepub.publish(ided_pose_array)

    

def callback(filtered_pose_array,predicted_pose_array):

    print("Entered callback")
    global firsttime
    firsttime=True     #When there are no predictions coming this should be made to false again

    #Predicted poses part
    predicted_xy_list=[]
    ids=[]

    for i in predicted_pose_array.poses:
        predicted_xy_list.append([i.pose.position.x,i.pose.position.y])
        ids.append(i.ID)

    #Filtered poses part
    filtered_xy_list=[]
    selected_xy_list=[]
    #final_ptime=filter_poses.header.stamp
    ided_pose_array=PoseIDArray()
    ided_pose_array.header.stamp=filtered_pose_array.header.stamp
    ided_pose_array.header.frame_id=ided_pose_array.header.frame_id

    for i in filtered_pose_array.poses:
        filtered_xy_list.append([i.position.x,i.position.y])

    
    print('Filtered pose List:')
    print(filtered_xy_list)
    print('Predicted pose List:')
    print(predicted_xy_list)
    cost=cost_matrix(filtered_xy_list,predicted_xy_list)

    #Solve the assignment problem
    row_indices, col_indices = linear_sum_assignment(cost)

    #Extract the optimal assignment
    assignment = [(row, col) for row, col in zip(row_indices, col_indices)]

    print("Optimal Assignment:")

    for row, col in assignment:
        print(f"Pose {filtered_xy_list[row]} in poses1, assigned to poses {predicted_xy_list[col]} in Poses2")
        
        #if(np.linalg.norm(np.array(filtered_xy_list[row])-np.array(predicted_xy_list[col]))<=0.7):

        selected_xy_list.append(filtered_xy_list[row])

        ided_pose=PoseID()
        ided_pose.pose.position.x=filtered_xy_list[row][0]
        ided_pose.pose.position.y=filtered_xy_list[row][1]
        ided_pose.ID=ids[col]

        ided_pose_array.poses.append(ided_pose)

        # else:
        #     print('Skipped because prediction and filtered measurement were too far apart')
        

    for k in selected_xy_list:
        if(k in filtered_xy_list):
            filtered_xy_list.remove(k)
            print("Got removed:",k)

    for j in filtered_xy_list:
        ided_pose=PoseID()
        ided_pose.pose.position.x=j[0]
        ided_pose.pose.position.y=j[1]
        ided_pose.ID=0
        print('Going to be created:',(0,j[0],j[1]))

        ided_pose_array.poses.append(ided_pose)

    measurepub.publish(ided_pose_array)
        
def main():

    global measurepub,firsttime
    rospy.init_node('Hungarian_Algorithm')
    firsttime=False
    
    pose_filtered_sub=rospy.Subscriber('/PoseFilteredLaser',PoseArray,callback_filtered_laser)

    # pose_predictions_sub=rospy.Subscriber('/PredictedPoses',PoseIDArray,callback_predicted)

    pose_filtered_sub = message_filters.Subscriber('/PoseFilteredLaser', PoseArray, queue_size=10)
    pose_predictions_sub = message_filters.Subscriber('/PredictedPoses', PoseIDArray, queue_size=10)

    #Debug this by checking if its actaully the problem with time sychronisaer. echo the above topics

    ts = message_filters.ApproximateTimeSynchronizer([pose_filtered_sub,pose_predictions_sub], 10, 0.1) #try varying time diff

    ts.registerCallback(callback)

    measurepub=rospy.Publisher('/Measurements',PoseIDArray,queue_size=10)

    rospy.spin()
        
  

if __name__ == '__main__':
    main()
    
