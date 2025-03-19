#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
from tracking.msg import PoseID, PoseIDArray

# Object Initialization
kalman_path1 = Path()
kalman_path2 = Path()

measured_path1 = Path()
measured_path2 = Path()

measured_trajectories = {}
kalman_trajectories = {}

# Functions
def pose_to_position(pose):
    return (pose.pose.position.x, pose.pose.position.y, pose.header.stamp.to_sec())

def update_trajectory(trajectory_dict, person_id, position):
    if person_id not in trajectory_dict:
        trajectory_dict[person_id] = []
    trajectory_dict[person_id].append(position)

def plot_trajectories():
    color_cycle = plt.cm.tab10.colors
    num_persons = len(measured_trajectories)
    num_rows = math.ceil(num_persons / 2)
    fig, axes = plt.subplots(num_rows + 1, 2, figsize=(12, 6 * (num_rows + 1)))
    axes = axes.flatten()

    for idx, (person_id, measured_traj) in enumerate(measured_trajectories.items()):
        kalman_traj = kalman_trajectories.get(person_id, [])

        if not kalman_traj:
            continue

        measured_color = color_cycle[idx % len(color_cycle)]
        kalman_color = color_cycle[(idx + 1) % len(color_cycle)]
        ax = axes[idx]

        if measured_traj:
            measured_x, measured_y, _ = zip(*measured_traj)
            ax.plot(measured_x, measured_y, label=f'Person {person_id} (Measured)', color=measured_color)
        if kalman_traj:
            kalman_x, kalman_y, _ = zip(*kalman_traj)
            ax.plot(kalman_x, kalman_y, label=f'Person {person_id} (Kalman)', color=kalman_color, linestyle='dashed')

        ax.set_xlabel('X (metres)')
        ax.set_ylabel('Y (metres)')
        ax.set_title(f'Trajectories of Person {person_id}')
        ax.legend()
        ax.grid(True)
        ax.plot(0, 0, 'o')

    plt.tight_layout()
    plt.show()

def kalman_callback(pose_array):
    for pose in pose_array.poses:
        position = pose_to_position(pose)
        update_trajectory(kalman_trajectories, pose.ID, position)

def measured_callback(pose_array):
    for pose in pose_array.poses:
        position = pose_to_position(pose)
        update_trajectory(measured_trajectories, pose.ID, position)

def main():
    rospy.init_node('Visualisation_Node')
    print('Visualisation started')

    rospy.Subscriber('/kalmanposeArray', PoseIDArray, kalman_callback)
    rospy.Subscriber('/Measurements', PoseIDArray, measured_callback)

    rospy.on_shutdown(plot_trajectories)
    rospy.spin()

if __name__ == '__main__':
    main()