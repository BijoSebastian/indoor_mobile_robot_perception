#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

# Global Variables
background_scan = None
pose_lidar_pub = None
first_time = True
max_range = 10.0  # Set this to the maximum range of your lidar

def polartorect(randtheta):
    return [randtheta[0] * np.cos(randtheta[1]), randtheta[0] * np.sin(randtheta[1])]

def filter_infs(scan):
    return np.array([min(r, max_range) if not np.isinf(r) else max_range for r in scan])

def callback(msg):
    global background_scan, first_time

    current_scan = filter_infs(np.array(msg.ranges))

    if first_time:
        background_scan = current_scan
        first_time = False
        rospy.loginfo("Background scan stored")
        return

    difference = np.abs(current_scan - background_scan)
    
    # Set a threshold for considering the difference as a significant change
    threshold = 0.2
    significant_indices = np.where(difference > threshold)[0]
    
    pts_r = current_scan[significant_indices]
    pts_ang = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))[significant_indices]

    newscan_rect = []
    for r, ang in zip(pts_r, pts_ang):
        if r < max_range:  # Ignore points at the maximum range
            [x_new, y_new] = polartorect([r, ang])
            newscan_rect.append([x_new, y_new])

    # Publish the points as PoseArray for visualization
    lidar_poses = PoseArray()
    lidar_poses.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
    for point in newscan_rect:
        pose = Pose()
        pose.position.x = point[0]
        pose.position.y = point[1]
        lidar_poses.poses.append(pose)
    
    pose_lidar_pub.publish(lidar_poses)

def main():
    global pose_lidar_pub

    rospy.init_node('bg_sub')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pose_lidar_pub = rospy.Publisher('/PoseLidar', PoseArray, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()
