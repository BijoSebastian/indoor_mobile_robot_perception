#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from sensor_msgs.msg import Image
import numpy as np

def rotation_matrix_to_quaternion(R):
    """Convert a 3x3 rotation matrix to a quaternion."""
    R_homogeneous = np.eye(4)
    R_homogeneous[:3, :3] = R
    q = tf_conversions.transformations.quaternion_from_matrix(R_homogeneous)
    return q

def broadcast_transform(msg):
    transformStamped = geometry_msgs.msg.TransformStamped()
    transformStamped.header.stamp = msg.header.stamp
    transformStamped.header.frame_id = "usb_cam"
    transformStamped.child_frame_id = "lidar"
    
    transformStamped.transform.translation.x = -0.02
    transformStamped.transform.translation.y = -0.05
    transformStamped.transform.translation.z = -0.06

    # Define the rotation matrix to flip the z-axis
    R_flip = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]  # Flip the z-axis
    ])
    
    # Additional rotations: roll, pitch, yaw
    roll = 0
    pitch = 3.14 / 2
    yaw = -3.14 / 2

    # Rotation matrices for roll, pitch, and yaw
    R_roll = tf_conversions.transformations.rotation_matrix(roll, [1, 0, 0])[:3, :3]
    R_pitch = tf_conversions.transformations.rotation_matrix(pitch, [0, 1, 0])[:3, :3]
    R_yaw = tf_conversions.transformations.rotation_matrix(yaw, [0, 0, 1])[:3, :3]

    # Combine all rotation matrices
    R_total = np.dot(R_flip,np.dot(R_yaw, np.dot(R_pitch,R_roll)))
    
    # Convert the combined rotation matrix to a quaternion
    q_rot = rotation_matrix_to_quaternion(R_total)

    transformStamped.transform.rotation.x = q_rot[0]
    transformStamped.transform.rotation.y = q_rot[1]
    transformStamped.transform.rotation.z = q_rot[2]
    transformStamped.transform.rotation.w = q_rot[3]

    br.sendTransform(transformStamped)

if __name__ == '__main__':
    try:
        rospy.init_node('tf_broadcaster_camtolid')
        br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/usb_cam/image_rect', Image, broadcast_transform)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
