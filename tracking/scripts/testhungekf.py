#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseArray, Pose
from tracking.msg import PoseIDArray, PoseID
from sensor_msgs.msg import LaserScan

class TimedPublisher:
    def __init__(self):
        rospy.init_node('timed_publisher')

        self.pose_pub = rospy.Publisher('/PoseFilteredLaser', PoseArray, queue_size=10)
        self.start_time = rospy.Time.now().to_sec()

        # Timer to publish /PoseFilteredLaser every second
        rospy.Timer(rospy.Duration(0.5), self.publish_pose_filtered_laser)

    def publish_pose_filtered_laser(self, event):
        elapsed_time = rospy.Time.now().to_sec() - self.start_time

        if elapsed_time > 15:
            return  # Stop after t=15

        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        radius1 = 1
        radius = 2  # Radius of the circular path
        angular_speed = 0.4  # radians per second
        theta = angular_speed * elapsed_time

        # Person 1 - circular motion
        pose1 = Pose()
        pose1.position.x = radius * math.cos(theta)
        pose1.position.y = radius * math.sin(theta)
        msg.poses.append(pose1)
        print(f'Person 1 position ({pose1.position.x:.2f}, {pose1.position.y:.2f})')

        # Person 2 - offset circular motion
        if 5 < elapsed_time < 11:
            pose2 = Pose()
            pose2.position.x = radius * math.cos(theta + math.pi/4)
            pose2.position.y = radius * math.sin(theta + math.pi/4)
            msg.poses.append(pose2)
            print(f'Person 2 position ({pose2.position.x:.2f}, {pose2.position.y:.2f})')

        self.pose_pub.publish(msg)

if __name__ == '__main__':
    TimedPublisher()
    rospy.spin()
