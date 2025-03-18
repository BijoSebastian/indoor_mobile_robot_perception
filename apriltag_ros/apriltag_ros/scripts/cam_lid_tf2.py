#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from sensor_msgs.msg import Image
import numpy as np

def broadcast_transform(msg):
    
    transformStamped.header.stamp = msg.header.stamp
    #Based on documentation
    # transformStamped.transform.translation.x = -0.06
    # transformStamped.transform.translation.y = -0.06
    # transformStamped.transform.translation.z = -0.02
    #Manually found
    transformStamped.transform.translation.x = 0.04
    transformStamped.transform.translation.y = 0.05
    transformStamped.transform.translation.z = -0.06

    #q_rot = tf_conversions.transformations.quaternion_from_euler(0,(np.pi/2),(-np.pi/2))#Increaser these until the axis align
    q_rot = tf_conversions.transformations.quaternion_from_euler((np.pi/2),(np.pi/2)-((np.pi/180)*3),(np.pi)) #Manually found
    transformStamped.transform.rotation.x = q_rot[0]
    transformStamped.transform.rotation.y = q_rot[1]
    transformStamped.transform.rotation.z = q_rot[2]
    transformStamped.transform.rotation.w = q_rot[3]

    br.sendTransform(transformStamped)
        

if __name__ == '__main__':
    try:
        rospy.init_node('tf_broadcaster_camtolid')
        br = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        transformStamped = geometry_msgs.msg.TransformStamped()

        transformStamped.header.frame_id = "usb_cam"
        transformStamped.child_frame_id = "lidar"

        rospy.Subscriber('/usb_cam/image_rect',Image,broadcast_transform)

        rate.sleep()
    except rospy.ROSInterruptException:
        pass
