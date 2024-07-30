#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import matplotlib.pyplot as plt
import cv2  # importing cv

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray
from nav_msgs.msg import Path
from tracking.msg import PoseIDArray,PoseID
from std_msgs.msg import Header
import time

from sensor_msgs.msg import LaserScan

#Object Initialization


#Functions

def coord_to_angle(x1,y1,x2,y2):
# Calculate the angle in radians using arctangent (atan2)
    angle_rad = math.atan2(y2-y1, x2-x1)

# Ensure the angle is within the range of 0 to 360 degrees
    angle_deg_positive = (angle_rad + 2*math.pi)%(2*math.pi)

    return angle_deg_positive

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

    def prediction(self):
    
        delt=self.ctime-self.ptime
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
        print('Addition to Xc:',np.matmul(k,y))
        #p_updated=np.matmul((person.I-np.matmul(k,person.H)),self.P)
        self.P=np.matmul((person.I-np.matmul(k,person.H)),self.P)
        self.iterations=0
        self.ptime=self.ctime
        self.ctime=meas[3][0]





def callback(msg):
    
    global trackingstarted,iterations,present_time,first_callback_received#,kalmanpredpose_array
    print("CALLBACK ACTIVATED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    


    prediction_time_nsec=msg.header.stamp.to_nsec()
    prediction_time_sec=prediction_time_nsec*(10**(-9))
    prediction_time_sec+=1
    predicted_time=rospy.Time.from_seconds(prediction_time_sec) #This is supposed to activaate the second iteration in HAnode 
    #There is bit of loss in precision due to above 4 lines

    present_time=msg.header.stamp

    p_time_nsec=msg.header.stamp.to_nsec()
    p_time_sec=p_time_nsec*(10**(-9))

    kalmanpose_array=PoseIDArray()
    kalmanpose_array.header= Header(stamp=present_time,frame_id='base_frame')
    # kalmanpredpose_array=PoseIDArray()
    # kalmanpredpose_array.header= Header(stamp=present_time,frame_id='base_frame') #supposed to be predicted_time
    pose_list=[]

    for t in msg.poses:
        pose=[t.ID,t.pose.position.x,t.pose.position.y,p_time_sec]
        pose_list.append(pose)

    print(pose_list)

    for i in pose_list:
        meas=i[1:4]
        for j in people:
            if(i[0]==j.id):
                #plt.scatter(i[1],i[2],color='red')
                print("UPDATING POSE OF ",j.id)
                meas_new=(j).heading_angle(meas)
                present_position=np.array([meas_new[0][0],meas_new[1][0]])
                prev_position=np.array([j.Xc[0][0],j.Xc[1][0]])
                if(np.linalg.norm(present_position-prev_position)<=0.7):
                    (j).measurement_update(meas_new)
                    print('ID:',j.id)
                    print('Xc:',j.Xc)
                    break
                else:
                    print('Actually didnt update cuz it was too far away. EKF is too lazy')
                    break
        if(i[0]==0 or i[0]!=j.id):
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

    for i in people:
        kalmanpose=PoseID()
        kalmanpose.ID=i.id
        kalmanpose.pose.position.x=i.Xc[0][0]
        kalmanpose.pose.position.y=i.Xc[1][0]
        kalmanpose_array.poses.append(kalmanpose)

    # for i in people:
    #     print("PREDICTING...")
    #     i.prediction()
    #     print('ID:',i.id)
    #     print('Xc:',i.Xc)
    #     predpose=PoseID()
    #     predpose.ID=i.id
    #     predpose.pose.position.x=i.Xc[0][0]
    #     predpose.pose.position.y=i.Xc[1][0]
    #     kalmanpredpose_array.poses.append(predpose)

    # if people:
    #     i=people[0]
    #     duration=i.ctime-i.ptime
    #     increment = rospy.Duration(duration)
    #     new_time = present_time + increment
    #     kalmanpredpose_array.header= Header(stamp=new_time,frame_id='base_frame') 

    kalman_pose_pub.publish(kalmanpose_array)

    if not first_callback_received:
        first_callback_received = True
    #kalman_predicted_pose_pub.publish(kalmanpredpose_array)

def publish_predictions():

    global first_callback_received,kalmanpredpose_array,time_elapsed,people
    #print('Periodic prediction!')
    # if not first_callback_received:
    #     continue
    
    
    kalmanpredpose_array=PoseIDArray()
    kalmanpredpose_array.header= Header(stamp=present_time,frame_id='base_frame') #supposed to be predicted_time
    #print("TIMER ACTIVATED PREDICTION")

    if people:
        for i in people:
            print("PREDICTING...")
            time_elapsed=(rospy.Time.now().to_nsec())*(10**(-9))-i.ctime
            print('rospy.get_time():',rospy.Time.now())
            print('i.ctime:',i.ctime)
            print('Time elapsed:',time_elapsed)
            i.prediction()
            print('ID:', i.id)
            print('Xc:', i.Xc)
            predpose = PoseID()
            predpose.ID = i.id
            predpose.pose.position.x = i.Xc[0][0]
            predpose.pose.position.y = i.Xc[1][0]
            kalmanpredpose_array.poses.append(predpose)

    if people:
        i=people[0]
        duration=time_elapsed
        increment = rospy.Duration(duration)
        new_time = present_time + increment
        kalmanpredpose_array.header= Header(stamp=new_time,frame_id='base_frame') 

    kalman_predicted_pose_pub.publish(kalmanpredpose_array)


def main():
    global kalman_pose_pub,kalman_predicted_pose_pub,people,time_elapsed,first_callback_received,present_time

    #Initial Condition  
    print('start')  
    rospy.init_node('Kalman_filter')

    people=[]
    time_elapsed=0
    first_callback_received=False
    present_time = rospy.Time.now()
    print('present_time:',present_time)
    
    
    measurement_sub = rospy.Subscriber('/Measurements', PoseIDArray, callback)

    scan_sub=rospy.Subscriber('/scan', LaserScan, scan_callback)

    kalman_pose_pub=rospy.Publisher('/kalmanposeArray',PoseIDArray,queue_size=10)

    kalman_predicted_pose_pub=rospy.Publisher('/PredictedPoses',PoseIDArray,queue_size=10)

    # rate = rospy.Rate(10)

    # while rospy.Time.now() == rospy.Time(0):
    #     rospy.sleep(0.1)

    # while not rospy.is_shutdown():
        
    #     #print('hello')
    #     publish_predictions()
    #     #print('heehe')
    # rate.sleep()

    #rospy.Timer(rospy.Duration(0.1), publish_predictions)
    

    rospy.spin()


if __name__ == '__main__':
    main()


#Use image callack