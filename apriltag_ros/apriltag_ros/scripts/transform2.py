#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from sensor_msgs.msg import Image

def broadcast_transform(msg):
    

    
    transformStamped.header.stamp = msg.header.stamp
    transformStamped.transform.translation.x = 0
    transformStamped.transform.translation.y = 0
    transformStamped.transform.translation.z = 0

    q_rot = tf_conversions.transformations.quaternion_from_euler(-3.14/2,0,-3.14/2)
    transformStamped.transform.rotation.x = q_rot[0]
    transformStamped.transform.rotation.y = q_rot[1]
    transformStamped.transform.rotation.z = q_rot[2]
    transformStamped.transform.rotation.w = q_rot[3]

    br.sendTransform(transformStamped)
        

if __name__ == '__main__':
    try:
        rospy.init_node('tf_broadcaster')
        br = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        transformStamped = geometry_msgs.msg.TransformStamped()

        transformStamped.header.frame_id = "world"
        transformStamped.child_frame_id = "usb_cam"

        rospy.Subscriber('/usb_cam/image_rect',Image,broadcast_transform)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
