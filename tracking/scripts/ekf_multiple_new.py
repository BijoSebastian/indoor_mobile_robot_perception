#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from scipy.optimize import linear_sum_assignment

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray
from nav_msgs.msg import Path
from tracking.msg import PoseIDArray,PoseID
from std_msgs.msg import Header
import time

#Object Initialization


#Functions

def coord_to_angle(x1,y1,x2,y2):
# Calculate the angle in radians using arctangent (atan2)
    angle_rad = math.atan2(y2-y1, x2-x1)

# Ensure the angle is within the range of 0 to 360 degrees
    angle_deg_positive = (angle_rad + 2*math.pi)%(2*math.pi)

    return angle_deg_positive


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

#Classes

class kalman_models:

    def __init__(self,object_type,list_of_objects):
        self.object_type=object_type
        self.list_of_objects=list_of_objects


class person:

    #Class variables
    #H matrix for states to measurement space
    H=np.array([[1,0,0,0,0],
            [0,1,0,0,0],
            [0,0,1,0,0]])
    #R matrix for noise associated with measurement
    #Originally
    # R=np.array([[0.1,0,0],
    #         [0,0.1,0],
    #         [0,0,50]])
    
    R=np.array([[0.1*(10**4),0,0],
            [0,0.1*(10**4),0],
            [0,0,50*(10**8)]])
    
    #Identity matrix
    I=np.identity(5)
    #Last id
    last_id=1

    def __init__(self,Xc,Xp,P,rev,id,iterations,ptime):
        self.Xc=Xc
        self.Xp=Xp
        self.P=P
        self.rev=rev
        self.id=id
        self.iterations=iterations
        self.ptime=ptime
        self.ctime=ptime

    #Functions pertaining to the model of a person

    def heading_angle(self,meas):
        revmeas=self.rev
        angle=coord_to_angle(self.Xc[0][0],self.Xc[1][0],meas[0],meas[1])
        prevangle=self.Xc[2][0]
        angle=(2*math.pi)*revmeas + angle

        #Make condition here to check if one revolution is done.
        if (abs(angle - prevangle) > (0.60*2*math.pi)):#If there is very large change in the heading direction.It is assumed that this is possible just about when one rev is done. Crossing of 360-0 boundry.
            print("HEREEE")
            if (angle < prevangle):
                revmeas += 1
            else:
                revmeas -= 1
            
            angle=(2*math.pi)*revmeas + angle
        
        self.rev=revmeas
        measnew=np.array([[meas[0]],
                        [meas[1]],
                        [angle],
                        [meas[2]]])
        return measnew

    def g(self,delt):
        #This is the state funciton

        xp=self.Xc[0][0]
        yp=self.Xc[1][0]
        thetap=self.Xc[2][0]
        vp=abs(self.Xc[3][0])
        wp=self.Xc[4][0]
        
        #The state equations
        xn=xp+((vp)*math.cos(thetap)*delt)
        yn=yp+((vp)*math.sin(thetap)*delt)
        thetan=(thetap+(wp*delt))
        vn=vp
        wn=wp

        x_pred=np.array([[xn],
                         [yn],
                         [thetan],
                         [vn],
                         [wn]])
        
        x_prev=self.Xc

        

        return x_pred,x_prev

        # self.Xp[0][0]=xp
        # self.Xp[1][0]=yp
        # self.Xp[2][0]=thetap
        # self.Xp[3][0]=vp
        # self.Xp[4][0]=wp
    
    def compute_G(self,delt):

        x=self.Xc[0][0]
        y=self.Xc[1][0]
        theta=self.Xc[2][0]
        v=abs(self.Xc[3][0])
        w=self.Xc[4][0]
        
        g1x=1
        g1y=0
        g1theta=-v*math.sin(theta)*delt
        g1v=math.cos(theta)*delt
        g1w=0

        g2x=0
        g2y=1
        g2theta=v*math.cos(theta)*delt
        g2v=math.sin(theta)*delt
        g2w=0

        g3x=0
        g3y=0
        g3theta=1
        g3v=0
        g3w=delt

        g4x=0
        g4y=0
        g4theta=0
        g4v=1
        g4w=0

        g5x=0
        g5y=0
        g5theta=0
        g5v=0
        g5w=1

        g=np.array([[g1x,g1y,g1theta,g1v,g1w],
                    [g2x,g2y,g2theta,g2v,g2w],
                    [g3x,g3y,g3theta,g3v,g3w],
                    [g4x,g4y,g4theta,g4v,g4w],
                    [g5x,g5y,g5theta,g5v,g5w]])
        
        return g

    def prediction(self,delt):
    
        #delt=self.ctime-self.ptime
        x_pred,x_prev=person.g(self,delt)
        G=person.compute_G(self,delt)
        p_pred=np.matmul(np.matmul(G,self.P),(G.transpose()))


        self.Xc=x_pred
        self.Xp=x_prev
        self.P=p_pred

    def hfxn(x):
        #This is function that converts states to measurement domain
        x_meas=np.array([[x[0][0]],
                     [x[1][0]],
                     [x[2][0]]])
    
        return x_meas

    def measurement_update(self,meas):
    
        y=meas[0:3]-person.hfxn(self.Xc) #Convert to measurement domain
        s=np.matmul(np.matmul(person.H,self.P),((person.H).transpose()))+person.R
        k=np.matmul(np.matmul(self.P,(person.H.transpose())),(np.linalg.inv(s)))
        self.Xc=self.Xc+np.matmul(k,y)
        self.P=np.matmul((person.I-np.matmul(k,person.H)),self.P)
        self.iterations=0
        self.ptime=self.ctime
        self.ctime=meas[3][0]





def callback(filtered_pose_array):
    
    global kalmanpredpose_array,predicted_list 
    
    predicted_xy_list=[]
    ids=[]

    #Make predictions as a global list
    for i in predicted_list:
        predicted_xy_list.append([i[0],i[1]])
        ids.append(i[2])

    #Filtered poses part
    filtered_xy_list=[]
    selected_xy_list=[]
    ided_pose = []
    ided_pose_list = []

    ided_posemsg_array=PoseIDArray()
    ided_posemsg_array.header.stamp=filtered_pose_array.header.stamp
    ided_posemsg_array.header.frame_id=ided_posemsg_array.header.frame_id
    #final_ptime=filter_poses.header.stamp
    

    for i in filtered_pose_array.poses:
        filtered_xy_list.append([i.position.x,i.position.y])

    
    print('Filtered pose List:')
    print(filtered_xy_list)
    print('Predicted List:')
    print(predicted_list)
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

        ided_pose = []
        ided_pose.append(filtered_xy_list[row][0])
        ided_pose.append(filtered_xy_list[row][1])
        ided_pose.append(ids[col])

        ided_pose_list.append(ided_pose)

        ided_posemsg=PoseID()
        ided_posemsg.pose.position.x=filtered_xy_list[row][0]
        ided_posemsg.pose.position.y=filtered_xy_list[row][1]
        ided_posemsg.ID=ids[col]

        ided_posemsg_array.poses.append(ided_posemsg)

        
        

    for k in selected_xy_list:
        if(k in filtered_xy_list):
            filtered_xy_list.remove(k)
            print("Got removed:",k)

    for j in filtered_xy_list:
        ided_pose = []
        ided_pose.append(j[0])
        ided_pose.append(j[1])
        ided_pose.append(0)
        print('Going to be created:',(j[0],j[1],0))

        ided_posemsg=PoseID()
        ided_posemsg.pose.position.x=j[0]
        ided_posemsg.pose.position.y=j[1]
        ided_posemsg.ID=0


        ided_pose_list.append(ided_pose)
        ided_posemsg_array.poses.append(ided_posemsg)

    present_time=filtered_pose_array.header.stamp

    p_time_nsec=filtered_pose_array.header.stamp.to_nsec()
    p_time_sec=p_time_nsec*(10**(-9))

    kalmanpose_array=PoseIDArray()
    kalmanpose_array.header= Header(stamp=present_time,frame_id='base_frame')

    pose_list=[]

    for t in ided_pose_list:
        pose=[t[2],t[0],t[1],p_time_sec]
        pose_list.append(pose)


    for i in pose_list:
        meas=i[1:4]
        for j in people:
            if(i[0]==j.id):
                #plt.scatter(i[1],i[2],color='red')
                print("UPDATING POSE OF ",j.id)
                meas_new=(j).heading_angle(meas)
                present_position=np.array([meas_new[0][0],meas_new[1][0]])
                prev_position=np.array([j.Xc[0][0],j.Xc[1][0]])
                (j).measurement_update(meas_new)
                print(f'Position of person {j.id} is {[j.Xc[0][0],j.Xc[1][0]]}')
                
        if(i[0]==0):
            print("NEW CREATED")
            Xc_new=np.array([[meas[0]],
                            [meas[1]],
                            [0],
                            [0],
                            [0]
                            ])
            Xp_new=np.array([[meas[0]],
                            [meas[1]],
                            [0],
                            [0],
                            [0]
                            ])
            P_new=np.array([[1000,0,0,0,0],
                [0,1000,0,0,0],
                [0,0,1000,0,0],
                [0,0,0,1000,0],
                [0,0,0,0,1000]])
            
            rev_new=0

            id_new=person.last_id+1
            person.last_id+=1
            iterations_new=0
            ptime=p_time_sec
            print('New ID:',id_new)
            people.append(person(Xc_new,Xp_new,P_new,rev_new,id_new,iterations_new,ptime))

    #Deletion part
    index=0                                  
    for k in people:
        
        if(k.id not in [row[0] for row in pose_list]):
            k.iterations+=1
            
        if(k.id not in [row[0] for row in pose_list] and k.iterations>10):#add and time_elapse<10
            print('Deleted ID number:',k.id)
            people.pop(index)

        index+=1

    #Making kalman pose array for publishing
    for i in people:
        kalmanpose=PoseID()
        kalmanpose.header.stamp=present_time
        kalmanpose.ID=i.id
        kalmanpose.pose.position.x=i.Xc[0][0]
        kalmanpose.pose.position.y=i.Xc[1][0]
        kalmanpose.pose.orientation.z=i.Xc[2][0]
        kalmanpose.pose.position.z=i.Xc[3][0] # I m using z position to store linear velocity
        kalmanpose.pose.orientation.x=i.Xc[4][0] # I m using x orientation to store angular velocity
        kalmanpose_array.poses.append(kalmanpose)

    #Time to do prediction

    current_time=filtered_pose_array.header.stamp
    current_time=current_time.to_nsec()*(10**(-9)) #Time in seconds

    kalmanpredpose_array=PoseIDArray()
    predicted_list = []

    if people:
        for i in people:
            print("PREDICTING...")
            time_elapsed=current_time-i.ptime
            i.prediction(time_elapsed)
            predpose = PoseID()
            predpose.ID = i.id
            predpose.pose.position.x = i.Xc[0][0]
            predpose.pose.position.y = i.Xc[1][0]
            print(f'Position of person {i.id} is {[i.Xc[0][0],i.Xc[1][0]]}')
            kalmanpredpose_array.poses.append(predpose)
            predicted_list.append([i.Xc[0][0],i.Xc[1][0],i.id])

        if people:
            i=people[0]
            duration=time_elapsed
            increment = rospy.Duration(duration)
            new_time = filtered_pose_array.header.stamp + increment
            kalmanpredpose_array.header= Header(stamp=new_time,frame_id='base_frame') 

        kalman_predicted_pose_pub.publish(kalmanpredpose_array)

    kalman_pose_pub.publish(kalmanpose_array)
    measurepub.publish(ided_posemsg_array)
    #kalman_predicted_pose_pub.publish(kalmanpredpose_array)

def main():
    global kalman_pose_pub,kalman_predicted_pose_pub,people, predicted_list, measurepub

    #Initial Condition    

    people=[]
    predicted_list =[]
    
    rospy.init_node('Kalman_filter')

    measurement_sub = rospy.Subscriber('/PoseFilteredLaser', PoseArray, callback,queue_size=10)

    kalman_pose_pub=rospy.Publisher('/kalmanposeArray',PoseIDArray,queue_size=10)

    kalman_predicted_pose_pub=rospy.Publisher('/PredictedPoses',PoseIDArray,queue_size=10)
    
    measurepub=rospy.Publisher('/Measurements',PoseIDArray,queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()

