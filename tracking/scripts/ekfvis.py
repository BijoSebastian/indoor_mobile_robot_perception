#!/usr/bin/env python3

# Required imports
import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use the TkAgg backend (or another suitable backend)
import matplotlib.pyplot as plt
import threading
import math
import signal
import sys
from tracking.msg import PoseID, PoseIDArray
from visualization_msgs.msg import Marker, MarkerArray

# Global dictionaries to store trajectories for each person.
measured_trajectories = {}
kalman_trajectories = {}

# Lock for safely updating the trajectory dictionaries.
from threading import Lock
traj_lock = Lock()

def pose_to_position(pose):
    # Convert a PoseID message to a tuple of (x, y, timestamp).
    return (pose.pose.position.x, pose.pose.position.y, pose.header.stamp.to_sec())

def pose_to_position_with_heading(pose):
    # Convert a PoseID message to a tuple of (x, y, heading, timestamp).
    return (pose.pose.position.x, pose.pose.position.y,pose.pose.orientation.z,pose.header.stamp.to_sec())


def update_trajectory(traj_dict, person_id, position):
    # Update the trajectory for a given person ID with the new position.
    # If the person ID does not exist in the dictionary, create a new entry.
    # Each entry is a list of tuples (x, y, timestamp) or (x, y, heading, timestamp).
    if person_id not in traj_dict:
        traj_dict[person_id] = []
    traj_dict[person_id].append(position)

def measured_callback(msg):
    # Callback function to handle incoming measurements.
    # It updates the measured trajectories with the new positions.
    print('Receiving measurements!')
    with traj_lock:
        for pose in msg.poses:
            position = pose_to_position(pose)
            update_trajectory(measured_trajectories, pose.ID, position)

def kalman_callback(msg):
    # Callback function to handle incoming Kalman filter messages.
    # It updates the Kalman trajectories with the new positions.
    # Each position includes the heading (orientation.z) and timestamp.
    print('Receiving kalman messages!')
    with traj_lock:
        for pose in msg.poses:
            position = pose_to_position_with_heading(pose)
            update_trajectory(kalman_trajectories, pose.ID, position)

def plot_trajectories():
    # Function to plot the trajectories of measured and Kalman-filtered positions.
    # It runs in a separate thread to continuously update the plot.
    plt.ion()
    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        ax.cla()
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("Trajectories: Measured (solid) vs Kalman (dashed)")
        color_cycle = plt.cm.tab10.colors

        with traj_lock:
            for idx, person_id in enumerate(sorted(measured_trajectories.keys())):
                color = color_cycle[idx % len(color_cycle)]
                meas_traj = measured_trajectories.get(person_id, [])
                kalman_traj = kalman_trajectories.get(person_id, [])

                if meas_traj:
                    xs_meas, ys_meas = zip(*[(x, y) for x, y, t in meas_traj])
                    ax.plot(xs_meas, ys_meas, color=color,
                            linewidth=2, label=f'Person {person_id} Measured')

                if kalman_traj:
                    xs_k, ys_k = zip(*[(x, y) for x, y,_, t in kalman_traj])
                    ax.plot(xs_k, ys_k, linestyle='dashed', color=color,
                            linewidth=2, label=f'Person {person_id} Kalman')
                    
                    #Plotting heading arrows

                    if len(kalman_traj) > 0:
                        print('hi')
                        skip = max (1, len(kalman_traj)//10)

                        arrow_data = kalman_traj[::skip]

                        for x, y, heading,_ in arrow_data :
                            dx = 0.3 * math.cos (heading)
                            dy = 0.3 * math.sin (heading)
                            ax.arrow(x, y, dx, dy, head_width = 0.01*40, head_length = 0.005*20, fc=color, ec=color)


        ax.legend(loc='best')
        ax.grid(True)
        plt.pause(0.1)

    plt.ioff()
    plt.show()

def shutdown_handler(signum, frame):
    print("\nShutting down gracefully...")
    rospy.signal_shutdown("KeyboardInterrupt")

def main():
    signal.signal(signal.SIGINT, shutdown_handler)  # Handle Ctrl+C

    rospy.init_node('trajectory_visualizer', anonymous=True)

    rospy.Subscriber('/Measurements', PoseIDArray, measured_callback)
    rospy.Subscriber('/globalkalmanposeArray', PoseIDArray, kalman_callback)

    plot_thread = threading.Thread(target=plot_trajectories)
    plot_thread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt")
    finally:
        plot_thread.join()
        print("Visualizer shut down cleanly.")

if __name__ == '__main__':
    main()
