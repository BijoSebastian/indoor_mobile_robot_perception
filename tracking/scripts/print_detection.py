#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tracking.msg import PoseIDArray  # Make sure this matches your custom message
from std_msgs.msg import Header
import tf.transformations as tf_trans


def kalmanpose_callback(msg):
    rospy.loginfo("Received from /kalmanposeArray:")
    for i, pose_id in enumerate(msg.poses):
        ori = pose_id.pose.orientation
        stamp = pose_id.header.stamp
        time_in_sec = stamp.to_sec()
        # Convert quaternion to Euler angles
        quaternion = [ori.x, ori.y, ori.z, ori.w]
        roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion)
        rospy.loginfo(f"Time:{time_in_sec}")
        rospy.loginfo(f"  Pose {i+1}:")
        rospy.loginfo(f"    ID: {pose_id.ID}")
        rospy.loginfo(f"    Position: x={pose_id.pose.position.x:.2f}, y={pose_id.pose.position.y:.2f}, z={pose_id.pose.position.z:.2f}")
        rospy.loginfo(f"    Orientation: yaw={yaw}")
    rospy.loginfo("--------------------------------------------------")

def main():
    rospy.init_node('kalman_listener', anonymous=True)

    
    rospy.Subscriber('/kalmanposeArray', PoseIDArray, kalmanpose_callback)

    rospy.loginfo("Listening to /kalmanposeArray...")
    rospy.spin()

if __name__ == '__main__':
    main()
