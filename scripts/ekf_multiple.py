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
    R=np.array([[0.1,0,0],
            [0,0.1,0],
            [0,0,50]])
    #Identity matrix
    I=np.identity(5)
    #Last id
    last_id=1

    def __init__(self,Xc,Xp,P,rev,id):
        self.Xc=Xc
        self.Xp=Xp
        self.P=P
        self.rev=rev
        self.id=id

    #Functions pertaining to the model of a person

    def heading_angle(self,meas):
        revmeas=self.rev
        angle=coord_to_angle(meas[0],meas[1],self.Xc[0][0],self.Xc[1][0])
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
                        [angle]])
        return measnew

    def g(self):
        #This is the state funciton

        xp=self.Xc[0][0]
        yp=self.Xc[1][0]
        thetap=self.Xc[2][0]
        vp=abs(self.Xc[3][0])
        wp=self.Xc[4][0]
        
        #The state equations
        xn=xp+(vp)*math.cos(thetap)
        yn=yp+(vp)*math.sin(thetap)
        thetan=(thetap+wp)
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
    
    def compute_G(self):

        x=self.Xc[0][0]
        y=self.Xc[1][0]
        theta=self.Xc[2][0]
        v=abs(self.Xc[3][0])
        w=self.Xc[4][0]
        
        g1x=1
        g1y=0
        g1theta=-v*math.sin(theta)
        g1v=math.cos(theta)
        g1w=0

        g2x=0
        g2y=1
        g2theta=v*math.cos(theta)
        g2v=math.sin(theta)
        g2w=0

        g3x=0
        g3y=0
        g3theta=1
        g3v=0
        g3w=1

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
    
        x_pred,x_prev=person.g(self)
        G=person.compute_G(self)
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
    
        y=meas-person.hfxn(self.Xc) #Convert to measurement domain
        s=np.matmul(np.matmul(person.H,self.P),((person.H).transpose()))+person.R
        k=np.matmul(np.matmul(self.P,(person.H.transpose())),(np.linalg.inv(s)))
        self.Xc=self.Xc+np.matmul(k,y)
        p_updated=np.matmul((person.I-np.matmul(k,person.H)),self.P)


#Initial Condition    
Xc1=np.array([[0],
              [0],
              [0],
              [1],
              [0]])

Xp1=np.array([[0],
              [0],
              [0],
              [0],
              [0]])

P1=np.array([[1000,0,0,0,0],
              [0,1000,0,0,0],
              [0,0,1000,0,0],
              [0,0,0,1000,0],
              [0,0,0,0,1000]])

rev1=0

id1=1

Xc2=np.array([[1],
              [1],
              [0],
              [0],
              [0]])

Xp2=np.array([[0],
              [0],
              [0],
              [0],
              [0]])

P2=np.array([[1000,0,0,0,0],
              [0,1000,0,0,0],
              [0,0,1000,0,0],
              [0,0,0,1000,0],
              [0,0,0,0,1000]])

rev2=0

id2=2

people=[person(Xc1,Xp1,P1,rev1,id1),person(Xc2,Xp2,P2,rev2,id2)]

iterations=0

for i in people:
    print("Initially...")
    print(i.id)
    print(i.Xc)
    print(i.P)

print("The measurements...")


def callback(msg):
    #global actualpath, kalmanpath
    #global X1,P1,H1,R1,I1,X2,P2,H2,R2,I2,XP1,YP1,THETAP1,XP2,YP2,THETAP2,
    global trackingstarted,iterations

    kalmanpose_array=PoseArray()
    kalmanpredpose_array=PoseIDArray()
    pose_array=[]
    
    for t in msg.poses:
        pose=[t.ID,t.pose.position.x,t.pose.position.z]
        pose_array.append(pose)


    for i in pose_array:
        meas=i[1:3]
        #print(i)
        #print(people)
        for j in people:
            if(i[0]==j.id):
                meas_new=(j).heading_angle(meas)
                (j).measurement_update(meas_new)
                iterations=0 #Would have to changee this. Each person have their own iterations variable
                break
        if(i[0]!=j.id):
            print("New created")
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
            
            people.append(person(Xc_new,Xp_new,P_new,rev_new,id_new))

    #Deletion part
    index=0
    for k in people:
        if(k.id not in [row[0] for row in pose_array]):
            iterations+=1
        if(k.id not in [row[0] for row in pose_array] and iterations>10):
            people.pop(index)
        index+=1

    for i in people:
        print("After updation...")
        print(i.id)
        print(i.Xc)
        print(i.P)

        kalmanpose=Pose()
        kalmanpose.position.x=i.Xc[0][0]
        kalmanpose.position.y=i.Xc[1][0]
        kalmanpose_array.poses.append(kalmanpose)

    for i in people:
        print("Predicting...")
        i.prediction()
        predpose=PoseID()
        predpose.ID=i.id
        predpose.pose.position.x=i.Xc[0][0]
        predpose.pose.position.z=i.Xc[1][0]
        kalmanpredpose_array.poses.append(predpose)
        print(i.id)
        print(i.Xc)
        print(i.P)
    
    kalmanposepub.publish(kalmanpose_array)
    kalmanpredictedposepub.publish(kalmanpredpose_array)

def main():
    global kalmanposepub,kalmanpredictedposepub
    rospy.init_node('Kalman_filter')
    pose_sub = rospy.Subscriber('/PoseArray', PoseIDArray, callback)

    kalmanposepub=rospy.Publisher('/kalmanposeArray',PoseArray,queue_size=20)
    kalmanpredictedposepub=rospy.Publisher('/PredictedPoses',PoseIDArray,queue_size=20)

    rospy.spin()


if __name__ == '__main__':
    main()

#make it publish the predicted positions
