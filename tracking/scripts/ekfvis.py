#!/usr/bin/env python3
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
    return (pose.pose.position.x, pose.pose.position.y, pose.header.stamp.to_sec())

def update_trajectory(traj_dict, person_id, position):
    if person_id not in traj_dict:
        traj_dict[person_id] = []
    traj_dict[person_id].append(position)

def measured_callback(msg):
    print('Receiving measurements!')
    with traj_lock:
        for pose in msg.poses:
            position = pose_to_position(pose)
            update_trajectory(measured_trajectories, pose.ID, position)

def kalman_callback(msg):
    print('Receiving kalman messages!')
    with traj_lock:
        for pose in msg.poses:
            position = pose_to_position(pose)
            update_trajectory(kalman_trajectories, pose.ID, position)

def plot_trajectories():
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
                meas_traj = measured_trajectories.get(person_id, [])
                kalman_traj = kalman_trajectories.get(person_id, [])

                if meas_traj:
                    xs_meas, ys_meas = zip(*[(x, y) for x, y, t in meas_traj])
                    ax.plot(xs_meas, ys_meas, color=color_cycle[idx % len(color_cycle)],
                            linewidth=2, label=f'Person {person_id} Measured')

                if kalman_traj:
                    xs_k, ys_k = zip(*[(x, y) for x, y, t in kalman_traj])
                    ax.plot(xs_k, ys_k, linestyle='dashed', color=color_cycle[idx % len(color_cycle)],
                            linewidth=2, label=f'Person {person_id} Kalman')

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
    rospy.Subscriber('/kalmanposeArray', PoseIDArray, kalman_callback)

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
