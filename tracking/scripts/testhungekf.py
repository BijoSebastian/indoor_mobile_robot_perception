#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Pose
from tracking.msg import PoseIDArray, PoseID
from sensor_msgs.msg import LaserScan

class TimedPublisher:
    def __init__(self):
        rospy.init_node('timed_publisher')

        self.pose_pub = rospy.Publisher('/PoseFilteredLaser', PoseArray, queue_size=10)
        #self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

        self.start_time = rospy.Time.now().to_sec()
        self.x = 1

        # Timer to publish /PoseFilteredLaser every second
        rospy.Timer(rospy.Duration(1), self.publish_pose_filtered_laser)

        #rospy.Timer(rospy.Duration(0.5), self.publish_scans)

    def publish_pose_filtered_laser(self, event):
        elapsed_time = rospy.Time.now().to_sec() - self.start_time

        if elapsed_time > 15:
            return  # Stop after t=15

        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # Dummy pose data (replace with actual data)
        #Person 1
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = 0
        msg.poses.append(pose)
        print(f'Person 1 position {self.x,0}')

        #Person 2
        if elapsed_time > 5 and elapsed_time < 11 :
            pose = Pose()
            pose.position.x = self.x
            pose.position.y = 1
            msg.poses.append(pose)
            print(f'Person 2 position {self.x,1}')
        

        self.pose_pub.publish(msg)
        #rospy.loginfo(f"Published /PoseFilteredLaser={self.x:.1f} at t={elapsed_time:.1f}")
        self.x+=1

    def publish_scans(self, event):
        elapsed_time = rospy.Time.now().to_sec() - self.start_time

        if elapsed_time > 25:
            return  

        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # Dummy pose data (replace with actual predictions)

        self.scan_pub.publish(msg)
        print('Scan published')
        #rospy.loginfo(f"Published /PredictedPoses={self.x+ 0.5:.1f} at t={elapsed_time:.1f}")

if __name__ == '__main__':
    TimedPublisher()
    rospy.spin()
