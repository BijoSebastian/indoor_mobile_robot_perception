#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose,PoseArray,PointStamped
from apriltag_ros.msg import PoseID, PoseIDArray

def get_apriltag_coordinates():
    rospy.init_node('tf_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    ground_truth_pub=rospy.Publisher('/ground_truth',PoseIDArray,queue_size=10)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        try:
            allpose_apriltag=PoseIDArray()
            allpose_apriltag.header.frame_id = "apriltag"
            allpose_apriltag.header.stamp = rospy.Time.now()

            if tfBuffer.can_transform('lidar', 'tag13', rospy.Time(0)):

                # Lookup the transform from lidar to apriltag
                transform = tfBuffer.lookup_transform('lidar', 'tag13', rospy.Time(0))
                transform_time = transform.header.stamp
                #rospy.loginfo("Transform from lidar to apriltag: %s", transform)            # Transform a point from apriltag frame to lidar frame
                point_in_apriltag = PointStamped()
                point_in_apriltag.header.frame_id = "apriltag"
                point_in_apriltag.header.stamp = transform_time
                point_in_apriltag.point.x = 0
                point_in_apriltag.point.y = 0
                point_in_apriltag.point.z = 0
                point_in_lidar = tf2_geometry_msgs.do_transform_point(point_in_apriltag, transform)
                #rospy.loginfo("Point in lidar frame: %s", point_in_lidar)

                pose_in_apriltag = PoseID()
                pose_in_apriltag.header.frame_id = "apriltag"
                pose_in_apriltag.header.stamp = transform_time
                pose_in_apriltag.ID=13
                pose_in_apriltag.pose.position.x = point_in_lidar.point.x
                pose_in_apriltag.pose.position.y = point_in_lidar.point.y
                pose_in_apriltag.pose.position.z = point_in_lidar.point.z

                allpose_apriltag.poses.append(pose_in_apriltag)
            
            if tfBuffer.can_transform('lidar', 'tag12', rospy.Time(0)):
                # Lookup the transform from lidar to apriltag
                transform = tfBuffer.lookup_transform('lidar', 'tag12', rospy.Time(0))
                #rospy.loginfo("Transform from lidar to apriltag: %s", transform)            # Transform a point from apriltag frame to lidar frame
                transform_time = transform.header.stamp
                point_in_apriltag = PointStamped()
                point_in_apriltag.header.frame_id = "apriltag"
                point_in_apriltag.header.stamp = transform_time
                point_in_apriltag.point.x = 0
                point_in_apriltag.point.y = 0
                point_in_apriltag.point.z = 0
                point_in_lidar = tf2_geometry_msgs.do_transform_point(point_in_apriltag, transform)
                #rospy.loginfo("Point in lidar frame: %s", point_in_lidar)
                pose_in_apriltag = PoseID()
                pose_in_apriltag.header.frame_id = "apriltag"
                pose_in_apriltag.header.stamp = transform_time
                pose_in_apriltag.ID=12
                pose_in_apriltag.pose.position.x = point_in_lidar.point.x
                pose_in_apriltag.pose.position.y = point_in_lidar.point.y
                pose_in_apriltag.pose.position.z = point_in_lidar.point.z

                allpose_apriltag.poses.append(pose_in_apriltag)
            allpose_apriltag.header.stamp = transform_time
            ground_truth_pub.publish(allpose_apriltag)
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        except Exception as error:
            print('ERROR:',error)
            rospy.logwarn("Transform not available")
            continue        
    rate.sleep()
if __name__ == '__main__':
    try:
        get_apriltag_coordinates()
    #except rospy.ROSInterruptException:
    except Exception as error:
        print('Error happen',error)
        pass


