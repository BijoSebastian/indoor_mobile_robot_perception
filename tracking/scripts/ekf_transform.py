#!/usr/bin/env python3

# Libraries to import
import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from scipy.optimize import linear_sum_assignment

import tf.transformations
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,PoseWithCovarianceStamped
from nav_msgs.msg import Path
from tracking.msg import PoseIDArray,PoseID
from std_msgs.msg import Header
import time
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

#Functions

def cost_matrix(poses1,poses2):

    # Calculates the cost matrix between two sets of poses.

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

def normalize_angle(angle):
    """Normalize an angle to be within [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def transform_frame1_to_frame2(person_frame1, frame1_orientation, frame1_translation):
    """
    Transforms a point from the frame1 frame to the frame2 frame.
    
    Parameters:
    person_frame1 (tuple): (x, y, z) position of the person in the frame1 frame.
    frame1_orientation (tuple): (roll, pitch, yaw) orientation of the frame1 relative to the frame2 in radians.
    frame1_translation (tuple): (tx, ty, tz) translation of the frame1 relative to the frame2.
    
    Returns:
    np.array: Transformed (x', y', z') position in the frame2 frame.
    """
    roll, pitch, yaw = frame1_orientation
    tx, ty, tz = frame1_translation
    
    # Rotation matrices for 3D transformation
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    R = R_z @ R_y @ R_x
    
    # Convert input point to numpy array
    person_frame1_np = np.array(person_frame1)
    
    # Apply rotation
    rotated_point = R @ person_frame1_np
    
    # Apply translation
    person_frame2_frame = rotated_point + np.array([tx, ty, tz])
    
    return person_frame2_frame


#Classes

class kalman_models:

    def __init__(self,object_type,list_of_objects):
        # Write EKF model for each object type, eventually
        # object_type can be 'person', 'car', etc.
        self.object_type=object_type
        self.list_of_objects=list_of_objects


class person:

    #Class variables
    #H matrix for states to measurement space
    H=np.array([[1,0,0,0,0],
            [0,1,0,0,0]])
    
    # Measurement noise covariance matrix
    # This is a diagonal matrix with large values for the noise in x and y measurements
    # Adjust these values based on the expected noise in your measurements
    R=np.array([[10000,0],
            [0,10000]])
    
    #Identity matrix
    I=np.identity(5)
    #Last id
    last_id=1

    def __init__(self,Xc,Xp,P,rev,id,iterations,ptime):
        self.Xc=Xc # Current state (x,y,theta,v,w)
        self.Xp=Xp # Previous state (x,y,theta,v,w)
        self.P=P # Covariance matrix
        self.rev=rev #Counts the number of revolutions. (Rudimentry)
        self.id=id # Unique ID for the person
        self.iterations=iterations # Counts the number of iterations without measurement update
        self.ptime=ptime # Previous time 
        self.ctime=ptime # Current time 

    #Functions pertaining to the model of a person

    def g(self,delt):
        #This is the state funciton

        xp=self.Xc[0][0]
        yp=self.Xc[1][0]
        thetap=self.Xc[2][0]
        vp=abs(self.Xc[3][0])
        wp=self.Xc[4][0]
        
        if abs(wp) > 1e-5 :
            # When wp is not zero, use the circular motion equations
            xn = xp + ((vp/wp) * (math.sin(thetap + wp * delt) - math.sin(thetap)))
            yn = yp + ((vp/wp) * (-math.cos(thetap + wp * delt) + math.cos(thetap)))
        else :
            # When wp is zero, use the linear motion equations
            xn=xp+((vp)*math.cos(thetap)*delt)
            yn=yp+((vp)*math.sin(thetap)*delt)
        
        thetan=normalize_angle(thetap+(wp*delt))
        vn=vp
        wn=wp

        x_pred=np.array([[xn],
                         [yn],
                         [thetan],
                         [vn],
                         [wn]])
        
        x_prev=self.Xc

        return x_pred,x_prev
    
    def compute_G(self,delt):

        # This function computes the Jacobian matrix G of the state transition function g
        x=self.Xc[0][0]
        y=self.Xc[1][0]
        theta=self.Xc[2][0]
        v=abs(self.Xc[3][0])
        w=self.Xc[4][0]
        
        if abs(w) > 1e-5 :

            # For xn = xp + ((vp/wp) * (math.sin(thetap + wp * delt) - math.sin(thetap)))

            g1x=1
            g1y=0
            g1theta = (v/w) * ( math.cos(theta + (w*delt)) - math.cos(theta) )
            g1v = (1/w) * ( math.sin(theta + (w*delt)) - math.sin(theta) )
            g1w = ((-v/(w**2)) * ( math.sin(theta + (w*delt)) - math.sin(theta) )) + (((v*delt)/w) * (math.cos(theta + w*delt)))

            # For yn = yp + ((vp/wp) * (-math.cos(thetap + wp * delt) + math.cos(thetap)))

            g2x = 0
            g2y = 1
            g2theta = (v/w) * ( math.sin(theta + (w*delt)) - math.sin(theta) )
            g2v = (1/w) * (-math.cos(theta + w * delt) + math.cos(theta))
            g2w = ((-v/(w**2)) * ( -math.cos(theta + (w*delt)) + math.cos(theta) )) + (((v*delt)/w) * (math.sin(theta + w*delt)))

        else :

            # For xn=xp+((vp)*math.cos(thetap)*delt)
            
            g1x=1
            g1y=0
            g1theta=-v*math.sin(theta)*delt
            g1v=math.cos(theta)*delt
            g1w=0
            
            # For yn=yp+((vp)*math.sin(thetap)*delt)

            g2x=0
            g2y=1
            g2theta=v*math.cos(theta)*delt
            g2v=math.sin(theta)*delt
            g2w=0
        
        # For thetan=normalize_angle(thetap+(wp*delt))

        g3x=0
        g3y=0
        g3theta=1
        g3v=0
        g3w=delt

        # For vn=vp

        g4x=0
        g4y=0
        g4theta=0
        g4v=1
        g4w=0

        # For wn=wp

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
    
        # This function performs the prediction step of the Kalman filter.

        x_pred,x_prev=self.g(delt)
        G=self.compute_G(delt)
        Q = np.diag([0.05, 0.05, 100, 0.1, 300]) # Process noise covariance matrix (Tunable parameters)

        p_pred = (G @ self.P @ G.T) + Q

        self.Xc=x_pred
        self.Xp=x_prev
        self.P=p_pred

    def hfxn(x):
        #This is function that converts states to measurement domain - (x,y)
        x_meas=np.array([[x[0][0]],
                     [x[1][0]]])
    
        return x_meas

    def measurement_update(self,meas):

        # This function performs the measurement update step of the Kalman filter.
    
        meas_new=np.array([[meas[0]],
                     [meas[1]]])
        y=np.array(meas_new)-person.hfxn(self.Xc) #Convert to measurement domain - (x,y)
        s=np.matmul(np.matmul(person.H,self.P),((person.H).transpose()))+person.R
        k=np.matmul(np.matmul(self.P,(person.H.transpose())),(np.linalg.inv(s)))
        self.Xc=self.Xc+np.matmul(k,y)
        self.Xc[2][0] = normalize_angle(self.Xc[2][0])
        self.P=np.matmul((person.I-np.matmul(k,person.H)),self.P)
        self.iterations=0
        self.ptime=self.ctime
        self.ctime=meas[2]


def pose_callback(robot_pose):
    #Here using the robot pose, the kalman pose needs to transformed to global frame.
    #publish it to /globalkalmanposearray topic using PoseIDArray msg

    #This function keeps publishing the transform from the robot frame to the base frame
    # This transform is needed to convert the local coordinates of the people to global coordinates
    global robot_position


    robot_position = robot_pose

    x = robot_pose.pose.pose.position.x
    y = robot_pose.pose.pose.position.y
    q = robot_pose.pose.pose.orientation
    (_,_,yaw) = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])

    t = TransformStamped()
    t.header.stamp = robot_pose.header.stamp
    t.header.frame_id = "map"
    t.child_frame_id = "base_frame"
    t.transform.translation.x = x/1000
    t.transform.translation.y = y/1000
    t.transform.translation.z = 0
    quat = tf.transformations.quaternion_from_euler(0,0,yaw)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    tf_br.sendTransform(t)


def callback(filtered_pose_array):

    # This function is called when a new filtered pose array is received.
    # It performs association, convert the measurements to global frame and then performs the Kalman filter update and publishes the results.
    
    global kalmanpredpose_array,predicted_list, robot_position
    
    predicted_xy_list=[]
    ids=[]
    #prev_robot_yaw = 0

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

    # Association part
    # We will use the Hungarian algorithm to find the optimal assignment between filtered poses and predicted poses
    cost=cost_matrix(filtered_xy_list,predicted_xy_list)

    #Solve the assignment problem
    row_indices, col_indices = linear_sum_assignment(cost)

    #Extract the optimal assignment
    assignment = [(row, col) for row, col in zip(row_indices, col_indices)]

    for row, col in assignment:

        selected_xy_list.append(filtered_xy_list[row])

        # Now the new measurements have IDs assigned to them
        # The IDs are taken from the predicted poses
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
        # Remove the selected poses from the filtered list
        if(k in filtered_xy_list):
            filtered_xy_list.remove(k)
            #print("Got removed:",k)

    for j in filtered_xy_list:
        # If there are any filtered poses that were not assigned to any predicted pose, we assign them a new ID of 0
        # This is the case when a new person is detected
        ided_pose = []
        ided_pose.append(j[0])
        ided_pose.append(j[1])
        ided_pose.append(0)

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

    measglobalposearray = []

    for i in pose_list:

        # Convert the measurements to global frame using the robot position

        meas_x = i[1]
        meas_y = i[2]

        measposestamp = PoseStamped()
        measposestamp.header.stamp = present_time
        measposestamp.header.frame_id = "lidar"
        #measpose.header= Header(stamp=rospy.Time.now(),frame_id='usb_cam')
        measposestamp.pose.position.x = meas_x
        measposestamp.pose.position.y = meas_y
        measposestamp.pose.position.z = 0
        try:
                    tf_msg = tf_buffer.lookup_transform("map","lidar",present_time,rospy.Duration(0.2))

                    measglobalpose = tf2_geometry_msgs.do_transform_pose(measposestamp,tf_msg)

                    print('Measpose:',measglobalpose)

                    measglobalpose_list = [i[0],measglobalpose.pose.position.x,measglobalpose.pose.position.y,p_time_sec]

                    measglobalposearray.append(measglobalpose_list)
                    #intermediateposearray.poses.append(localpose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)

    pose_list = measglobalposearray

    for i in pose_list:
        meas=i[1:4]
        for j in people:
            if(i[0]==j.id):
                # This is the case when a person is already detected
                print("UPDATING POSE OF ",j.id)
                print('Meas:',meas)
                #meas_new=(j).heading_angle(meas)
                present_position=np.array([meas[0],meas[1]])
                prev_position=np.array([j.Xc[0][0],j.Xc[1][0]])
                (j).measurement_update(meas)
                #j.ptime = present_time
                print(f'Position of person {j.id} is {[j.Xc[0][0],j.Xc[1][0]]}')
                print(f'Heading of person {j.id} is {j.Xc[2][0]}')
                print(f'Velocity of person {j.id} is {[j.Xc[3][0],j.Xc[4][0]]}')
                
        if(i[0]==0):
            # This is the case when a new person is detected
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
            ptime = p_time_sec
            print('New ID:',id_new)
            people.append(person(Xc_new,Xp_new,P_new,rev_new,id_new,iterations_new,ptime))

    #Deletion part
    # We will delete the people who have not been updated for more than 10 iterations
    index=0                                  
    for k in people:
        
        if(k.id not in [row[0] for row in pose_list]):
            k.iterations+=1
            
        if(k.id not in [row[0] for row in pose_list] and k.iterations>10):#add and time_elapse<10
            print('Deleted ID number:',k.id)
            people.pop(index)

        index+=1

    #Making kalman pose array for publishing
    # Publishing the current state of each person in the kalmanpose_array
    for i in people:
        kalmanpose=PoseID()
        kalmanpose.header.stamp=rospy.Time.from_sec(i.ptime)
        kalmanpose.ID=i.id
        kalmanpose.pose.position.x=i.Xc[0][0]
        kalmanpose.pose.position.y=i.Xc[1][0]
        kalmanpose.pose.orientation.z=i.Xc[2][0]
        kalmanpose.pose.position.z=i.Xc[3][0] # I m using z position to store linear velocity
        kalmanpose.pose.orientation.x=i.Xc[4][0] # I m using x orientation to store angular velocity
        kalmanpose_array.poses.append(kalmanpose)

    #Time to do prediction
    # Get the current time in seconds
    # This is used to calculate the time elapsed since the last measurement update


    current_time=filtered_pose_array.header.stamp
    current_time=current_time.to_nsec()*(10**(-9)) #Time in seconds

    kalmanpredpose_array=PoseIDArray()
    predicted_list = []


    #print('robot_x:',robot_x)

    if people:
        for i in people:
            # For each person tracked, we will perform the prediction step of the Kalman filter
            # We will also publish the predicted pose in the kalmanpredpose_array
            print("PREDICTING...")
            time_elapsed=current_time-i.ptime #Time in seconds
            i.prediction(time_elapsed)
            predpose = PoseID()
            predpose.ID = i.id
            predpose.pose.position.x = i.Xc[0][0]
            predpose.pose.position.y = i.Xc[1][0]
            print(f'Position of person {i.id} is {[i.Xc[0][0],i.Xc[1][0]]}')
            print(f'Heading of person {i.id} is {i.Xc[2][0]}')
            print(f'Velocity of person {i.id} is {[i.Xc[3][0],i.Xc[4][0]]}')
            kalmanpredpose_array.poses.append(predpose)
            predicted_list.append([i.Xc[0][0],i.Xc[1][0],i.id])

        if people:
            i=people[0]
            duration=time_elapsed
            # This is used to calculate the time elapsed since the last measurement update
            # We will use this to update the header stamp of the kalmanpredpose_array
            increment = rospy.Duration(duration)
            new_time = filtered_pose_array.header.stamp + increment
            kalmanpredpose_array.header= Header(stamp=new_time,frame_id='base_frame')

            globalposearray = PoseIDArray()
            globalposearray.header= Header(stamp=filtered_pose_array.header.stamp,frame_id='map')

            for human in people:
                # Publishing the global pose of each person in the globalposearray in mm
                local_yaw=human.Xc[2][0]
                try:
                    globalposeid = PoseID()
                    globalposeid.ID = human.id
                    globalposeid.pose.position.x = human.Xc[0][0]*1000
                    globalposeid.pose.position.y = human.Xc[1][0]*1000
                    yaw = local_yaw
                    globalposeid.pose.orientation.z = yaw
                    globalposeid.pose.position.z= human.Xc[3][0] # I m using z position to store linear velocity
                    globalposeid.pose.orientation.x=human.Xc[4][0] # I m using z position to store angular velocity

                    globalposearray.poses.append(globalposeid)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("TF lookup failed: %s", e)

        kalman_predicted_pose_pub.publish(kalmanpredpose_array)
        global_kalman_pose_pub.publish(globalposearray)
        kalman_pose_pub.publish(kalmanpose_array)
        measurepub.publish(ided_posemsg_array)


def main():
    global kalman_pose_pub,kalman_predicted_pose_pub,people, predicted_list, measurepub, robot_position,global_kalman_pose_pub,intermediate_kalman_pose_pub
    global tf_buffer, tf_br

    #Initial Condition    

    people=[] # List to hold all the people objects
    predicted_list =[] # List to hold the predicted poses of people
    robot_position = PoseWithCovarianceStamped() # To hold the robot pose
    
    rospy.init_node('Kalman_filter')

    measurement_sub = rospy.Subscriber('/PoseFilteredLaser', PoseArray, callback,queue_size=1) # This is the topic where the filtered poses are published

    robot_pose_sub = rospy.Subscriber('/pose_ekf', PoseWithCovarianceStamped, pose_callback,queue_size=1) # This is the topic where the robot pose is published

    kalman_pose_pub=rospy.Publisher('/kalmanposeArray',PoseIDArray,queue_size=1) # This is the topic where the kalman poses are published

    global_kalman_pose_pub=rospy.Publisher('/globalkalmanposeArray',PoseIDArray,queue_size=1) # This is the topic where the global kalman poses are published

    kalman_predicted_pose_pub=rospy.Publisher('/PredictedPoses',PoseIDArray,queue_size=1) # This is the topic where the predicted poses are published
    
    measurepub=rospy.Publisher('/Measurements',PoseIDArray,queue_size=1) 

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_br = tf2_ros.TransformBroadcaster()

    # Static transform broadcaster to publish the transform from the base frame to the lidar frame

    static_br = tf2_ros.StaticTransformBroadcaster()
    rospy.sleep(0.1)

    static_t = TransformStamped()
    static_t.header.stamp = rospy.Time.now()
    static_t.header.frame_id = "base_frame"
    static_t.child_frame_id = "lidar"

    static_t.transform.translation.x = 0
    static_t.transform.translation.y = 0
    static_t.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(0,0,math.pi)
    static_t.transform.rotation.x = quat[0]
    static_t.transform.rotation.y = quat[1]
    static_t.transform.rotation.z = quat[2]
    static_t.transform.rotation.w = quat[3]

    static_br.sendTransform(static_t)



    rospy.spin()


if __name__ == '__main__':
    main()

