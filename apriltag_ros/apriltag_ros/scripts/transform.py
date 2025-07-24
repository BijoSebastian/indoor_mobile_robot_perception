#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def broadcast_transform():
    rospy.init_node('tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    transformStamped = geometry_msgs.msg.TransformStamped()

    transformStamped.header.frame_id = "world"
    transformStamped.child_frame_id = "usb_cam"

    while not rospy.is_shutdown():
        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.transform.translation.x = 0
        transformStamped.transform.translation.y = 0
        transformStamped.transform.translation.z = 0

        q_rot = tf_conversions.transformations.quaternion_from_euler(-3.14/2,0,-3.14/2)
        transformStamped.transform.rotation.x = q_rot[0]
        transformStamped.transform.rotation.y = q_rot[1]
        transformStamped.transform.rotation.z = q_rot[2]
        transformStamped.transform.rotation.w = q_rot[3]

        br.sendTransform(transformStamped)
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
