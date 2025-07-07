#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tracking.msg import PoseIDArray  # Make sure this matches your custom message
from std_msgs.msg import Header
import tf.transformations as tf_trans

def poseekf_callback(msg):
    
    ori = msg.pose.pose.orientation
    # Convert quaternion to Euler angles
    quaternion = [ori.x, ori.y, ori.z, ori.w]
    roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion)

    rospy.loginfo("Received from /poseekf:")
    rospy.loginfo(f"  Time: {msg.header.stamp.to_sec():.3f}")
    rospy.loginfo(f"  Position: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}, z={msg.pose.pose.position.z:.2f}")
    rospy.loginfo(f"  Yaw = {yaw}")
    rospy.loginfo("--------------------------------------------------")


def main():
    rospy.init_node('pose_ekf_listener', anonymous=True)

    rospy.Subscriber('/pose_ekf', PoseWithCovarianceStamped, poseekf_callback)
    

    rospy.loginfo("Listening to /poseekf")
    rospy.spin()

if __name__ == '__main__':
    main()
