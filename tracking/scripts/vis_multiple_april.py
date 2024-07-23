#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use the TkAgg backend (or another suitable backend)
import matplotlib.pyplot as plt
import math

from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,PoseStamped,PoseArray,PointStamped
from nav_msgs.msg import Path
from tracking.msg import PoseID,PoseIDArray

#Object Initialization


apriltag_path1=Path()
apriltag_path2=Path()


actual_trajectories = {}


#Variable Initialization
actualpath = []


#Functions

def pose_to_position(pose):
    """
    Convert PoseArray message to a list of (x, y) positions.
    """
    return (pose.position.x, pose.position.y)

def pose_to_position_april(pose):
    """
    Convert PoseArray message to a list of (x, y) positions.
    """
    # return (pose.position.z, pose.position.y)
    return (-pose.position.z, -pose.position.x)

def update_trajectory(trajectory_dict, person_id, position):
    """
    Update trajectory of a specific person in the dictionary.
    """
    if person_id not in trajectory_dict:
        trajectory_dict[person_id] = []
    trajectory_dict[person_id].append(position)

def plot_trajectories():
    """
    Plot trajectories of all persons.
    """
    # include the true path (distance from camera) of a person walking where a person holds april tag and starts walking

    #plt.figure()
    color_cycle = plt.cm.tab10.colors
    num_persons = len(actual_trajectories)
    num_rows = math.ceil(num_persons / 2)  # Adjust the number of rows based on the number of persons
    fig, axes = plt.subplots(num_rows, 2, figsize=(12, 6 * num_rows))  # Create subplots
    axes = axes.flatten()
    #for person_id, measured_traj in measured_trajectories.items():
    for idx, (person_id, actual_traj) in enumerate(actual_trajectories.items()):

        print(person_id)
        print(actual_trajectories)
        
        actual_traj = actual_trajectories.get(person_id, [])
        if(len(actual_traj)==0):
            continue
        
        actual_color = color_cycle[(idx + 2) % len(color_cycle)] # Actual path color april tag distance measurement
        ax = axes[idx]
        x, y = zip(*actual_traj)
        ax.plot(x, y, label=f'Person {person_id} (Actual)', color=actual_color, linestyle='dotted')
        #plt.plot(x, y, label=f'Person {person_id} (Measured)', color='blue')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title(f'Trajectories of Person {person_id}')
        ax.legend()
        ax.grid(True)
        #plt.plot(x, y, label=f'Person {person_id} (Kalman)', color='red', linestyle='dashed')
        ax.plot(0,0,'o')

    plt.tight_layout()
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.title('Trajectories of Persons')
    # plt.legend()
    #plt.grid(True)
    #plt.savefig('person_path.png')
    
    plt.show()
    plt.pause(10)

    # plt.pause(0.001)
    # plt.clf()
    # plt.close()
    #plt.show()

    

    
def apriltag_callback(pose_array):
    #visualization_markers(pose_array,apriltag_markerarray_pub)
    for pose in pose_array.poses:
        #Assign Apriltag id as person id
        person_id = 2
        position = pose_to_position_april(pose)
        update_trajectory(actual_trajectories, person_id, position)
        print('UPDATING!')

def apriltag_callback_point(point):
    #visualization_markers(pose_array,apriltag_markerarray_pub)
    
    #Assign Apriltag id as person id
    person_id = 2
    position = (point.point.x,point.point.y)
    update_trajectory(actual_trajectories, person_id, position)
    print('UPDATING!')


def measured_pathmaker(pose,measured_path,measured_path_pub):

    x = pose.position.x 
    y = pose.position.y

    if(not math.isnan(x) and not math.isnan(y)):  
        measured_path.header.frame_id="map"
        measured_path.header.stamp=rospy.Time.now()
        pose_current = PoseStamped()
        pose_current.pose.position.x = x
        pose_current.pose.position.y = y
        pose_current.pose.position.z = 0
        measured_path.poses.append(pose_current)

        measured_path_pub.publish(measured_path)



def main():
    global apriltag_markerarray_pub
    rospy.init_node('Visualisation_Node_April')
    print('Visualisation started')
    
    #1apriltag_pose_sub = rospy.Subscriber('/lidar_apriltag_pose',PoseArray,apriltag_callback)
    apriltag_pose_sub = rospy.Subscriber('/ground_truth',PointStamped,apriltag_callback_point)

    apriltag_path_pub1=rospy.Publisher('/apriltag_path1',Path,queue_size=20)
    apriltag_path_pub2=rospy.Publisher('/apriltag_path2',Path,queue_size=20)    

    apriltag_markerarray_pub=rospy.Publisher('/apriltag_markers',MarkerArray,queue_size=20)

    rospy.on_shutdown(plot_trajectories)

    #rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()

#The number of circles keeps changing