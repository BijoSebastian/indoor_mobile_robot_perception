#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
#from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

from scipy.optimize import linear_sum_assignment

import math
import joblib

lidar_to_pixel = joblib.load('/home/winston/catkin_ws/src/tracking/scripts/linear_regression_model.joblib')



final_poses=[]
prediction_poses=[]
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


def callback_lidar(msg):

    global lidar_poses,lidpix_poses

    lidar_poses=[]
    lidpix_poses=[]
    predicted=[]
    
    for i in msg.poses:
        print(i)
        lidar_poses.append([i.position.x,i.position.z])

    
    for j in lidar_poses:
        #print(j)
        xL=j[0]
        zL=j[1]
        poly = PolynomialFeatures(degree=5)
        translidpose = poly.fit_transform([j])
        predicted=lidar_to_pixel.predict(translidpose)
        print('predicted:',predicted)
        #predicted[0][0]= (0.2 -1.37*xL -0.34*zL-1.51*(xL/zL)+0*(zL/xL)-0.22*(xL**2)-0.22*(zL**2)-0.95*(xL*zL))*1000
        predicted[0][0]= (318.52-1087*xL -1415*(xL/zL)-758*(xL*zL))

        lidpix_poses.append(predicted)


    

    

def callback_cam(msg):

    cam_poses=[]
    
    for i in msg.poses:
        cam_poses.append([i.position.x,i.position.y])

    cost=cost_matrix(lidpix_poses,cam_poses)

    #Solve the assignment problem
    row_indices, col_indices = linear_sum_assignment(cost)

    # # Extract the optimal assignment
    assignment = [(row, col) for row, col in zip(row_indices, col_indices)]

    print("Optimal Assignment:")
    print(len(cam_poses))
    print(len(lidar_poses))
    for row, col in assignment:
        print(f"Pose {lidar_poses[row]} in poses1, assigned to poses {cam_poses[col]} in Poses2")
        print("lidpix poses:")
        print(lidpix_poses[row])
        #final_poses.append(lidar_poses[row])


# def callback_prediction(msg):
    
#     for i in msg.poses:
#         prediction_poses.append([i.position.x,i.position.y])

#     cost=cost_matrix(final_poses,prediction_poses)

#     #Solve the assignment problem
#     row_indices, col_indices = linear_sum_assignment(cost)

#     # # Extract the optimal assignment
#     assignment = [(row, col) for row, col in zip(row_indices, col_indices)]

#     print("Optimal Assignment:")
#     for row, col in assignment:
#         print(f"Pose {final_poses[row]} in poses1, assigned to poses {prediction_poses[col]} in Poses2")
#         final_poses[row].append(prediction_poses[col][2]) #adding ids to the measurements

        #after this, those which werent mapped should be given new ids

    
        
    




def main():

    global posepub,visualpub
    rospy.init_node('Hungarian_Algorithm')
    

    pose_lidar_sub=rospy.Subscriber('/PoseLidar',PoseArray,callback_lidar)

    pose_cam_sub=rospy.Subscriber('/detection_topic',PoseArray,callback_cam)

    # pose_cam_sub=rospy.Subscriber('/PosePrediction',PoseArray,callback_prediction)

    visualpub=rospy.Publisher('/Pose',Pose,queue_size=10)

    
    rospy.spin()
        
  

if __name__ == '__main__':
    main()
    

# '''Kalman filter to find prediciton of bounding positions after t+1. Then perform linear sum assignment'''


'''Optimal Assignment:
31
1474
Pose [-2.5184807777404785, 2.7920000553131104] in poses1, assigned to poses [515.0, 247.0] in Poses2
Pose [-2.5184807777404785, 2.7920000553131104] in poses1, assigned to poses [163.0, 281.0] in Poses2
Pose [2.296285629272461, 1.99399995803833] in poses1, assigned to poses [510.0, 248.0] in Poses2
Pose [2.296285629272461, 1.99399995803833] in poses1, assigned to poses [169.0, 283.0] in Poses2
Pose [2.7842981815338135, 4.711999893188477] in poses1, assigned to poses [501.0, 249.0] in Poses2
Pose [2.650951385498047, 3.124084949493408] in poses1, assigned to poses [183.0, 279.0] in Poses2
Pose [-2.5184807777404785, 2.7920000553131104] in poses1, assigned to poses [489.0, 245.0] in Poses2
Pose [2.296285629272461, 1.99399995803833] in poses1, assigned to poses [187.0, 281.0] in Poses2
Pose [-0.8714384436607361, 4.5279998779296875] in poses1, assigned to poses [479.0, 246.0] in Poses2
Pose [0.9411794543266296, 5.432000160217285] in poses1, assigned to poses [186.0, 281.0] in Poses2
Pose [2.980374574661255, 5.095999717712402] in poses1, assigned to poses [470.0, 242.0] in Poses2
Pose [2.9237303733825684, 5.5839996337890625] in poses1, assigned to poses [183.0, 278.0] in Poses2
Pose [-2.9977786540985107, 5.192000389099121] in poses1, assigned to poses [468.0, 240.0] in Poses2
Pose [-2.7668442726135254, 3.694000005722046] in poses1, assigned to poses [181.0, 274.0] in Poses2
Pose [-2.7668442726135254, 3.694000005722046] in poses1, assigned to poses [463.0, 241.0] in Poses2
Pose [-0.9367972612380981, 4.944000244140625] in poses1, assigned to poses [184.0, 272.0] in Poses2
Pose [2.650951385498047, 3.124084949493408] in poses1, assigned to poses [462.0, 242.0] in Poses2
Pose [0.8627488613128662, 4.935999870300293] in poses1, assigned to poses [187.0, 268.0] in Poses2
Pose [2.6710095405578613, 3.1419999599456787] in poses1, assigned to poses [460.0, 239.0] in Poses2
Pose [-0.8714384436607361, 4.5279998779296875] in poses1, assigned to poses [460.0, 239.0] in Poses2
Pose [2.6543400287628174, 3.124376058578491] in poses1, assigned to poses [189.0, 268.0] in Poses2
Pose [-0.8714384436607361, 4.5279998779296875] in poses1, assigned to poses [455.0, 239.0] in Poses2
Pose [0.9324649572372437, 5.380000114440918] in poses1, assigned to poses [191.0, 270.0] in Poses2
Pose [2.6710095405578613, 3.1419999599456787] in poses1, assigned to poses [448.0, 238.0] in Poses2
Pose [-3.028010129928589, 4.871181011199951] in poses1, assigned to poses [448.0, 238.0] in Poses2
Pose [-2.8801326751708984, 5.260000228881836] in poses1, assigned to poses [201.0, 268.0] in Poses2
Pose [0.9237504601478577, 5.303999900817871] in poses1, assigned to poses [437.0, 240.0] in Poses2
Pose [-0.9280827641487122, 4.888000011444092] in poses1, assigned to poses [437.0, 240.0] in Poses2
Pose [2.296285629272461, 1.9909999370574951] in poses1, assigned to poses [205.0, 265.0] in Poses2
Pose [-0.8714384436607361, 4.5279998779296875] in poses1, assigned to poses [431.0, 240.0] in Poses2
Pose [0.9237504601478577, 5.308000087738037] in poses1, assigned to poses [215.0, 265.0] in Poses2
'''
