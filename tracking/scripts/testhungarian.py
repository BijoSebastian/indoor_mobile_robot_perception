#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Pose
from tracking.msg import PoseIDArray, PoseID

class TimedPublisher:
    def __init__(self):
        rospy.init_node('timed_publisher')

        self.pose_pub = rospy.Publisher('/PoseFilteredLaser', PoseArray, queue_size=10)
        self.pred_pub = rospy.Publisher('/PredictedPoses', PoseIDArray, queue_size=10)

        self.start_time = rospy.Time.now().to_sec()
        self.x = 1

        # Timer to publish /PoseFilteredLaser every second
        rospy.Timer(rospy.Duration(1), self.publish_pose_filtered_laser)

        # Timer to publish /PredictedPoses every second (but only from t=5 to t=10)
        #rospy.Timer(rospy.Duration(1), self.publish_predicted_poses)

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

        #Person 2
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = 1
        msg.poses.append(pose)
        

        self.pose_pub.publish(msg)
        rospy.loginfo(f"Published /PoseFilteredLaser={self.x:.1f} at t={elapsed_time:.1f}")
        self.x+=1

    def publish_predicted_poses(self, event):
        elapsed_time = rospy.Time.now().to_sec() - self.start_time

        if elapsed_time < 5 or elapsed_time > 10:
            return  # Only publish in range [5, 10]

        msg = PoseIDArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # Dummy pose data (replace with actual predictions)
        #Person 1
        pose_id = PoseID()
        pose_id.pose.position.x = self.x + 0.5
        pose_id.pose.position.y = 0
        pose_id.ID = 1
        msg.poses.append(pose_id)

        #Person 2
        pose_id = PoseID()
        pose_id.pose.position.x = self.x + 0.5
        pose_id.pose.position.y = 1
        pose_id.ID = 2
        msg.poses.append(pose_id)

        self.pred_pub.publish(msg)
        rospy.loginfo(f"Published /PredictedPoses={self.x+ 0.5:.1f} at t={elapsed_time:.1f}")

if __name__ == '__main__':
    TimedPublisher()
    rospy.spin()
