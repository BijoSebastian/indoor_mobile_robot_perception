#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Pose
from tracking.msg import PoseIDArray, PoseID

class TimedPublisher:
    def __init__(self):
        rospy.init_node('timed_publisher')

        self.pose_pub = rospy.Publisher('/Measurements', PoseIDArray, queue_size=10)

        self.start_time = rospy.Time.now().to_sec()
        self.x = 0.5
        self.ftime1 = True
        self.ftime2 = True

        # Timer to publish /Measurements every second
        rospy.Timer(rospy.Duration(0.5), self.publish_measurements)

        # Timer to publish /PredictedPoses every second (but only from t=5 to t=10)
        #rospy.Timer(rospy.Duration(1), self.publish_predicted_poses)

    def publish_measurements(self, event):
        elapsed_time = rospy.Time.now().to_sec() - self.start_time

        if elapsed_time > 25:
            return  # Stop after t=25

        msg = PoseIDArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # Dummy pose data (replace with actual data)
        if elapsed_time < 15 :
            #Person 1
            pose = PoseID()
            pose.ID = 2
            #First time, measurements come with default IDs. 
            if self.ftime1 == True :
                pose.ID = 0
                self.ftime1 = False
            pose.pose.position.x = self.x
            pose.pose.position.y = 0
            print(f'Person 1 position {self.x,0}')
            msg.poses.append(pose)

        if elapsed_time >5 :
            #Person 2
            pose = PoseID()
            pose.ID = 3
            #First time, measurements come with default IDs. (Maybe I should do ID handling in Hungarian node)
            if self.ftime2 == True :
                pose.ID = 0
                self.ftime2 = False
            pose.pose.position.x = self.x
            pose.pose.position.y = 1
            print(f'Person 2 position {self.x,1}')
            msg.poses.append(pose)
        

        self.pose_pub.publish(msg)
        #rospy.loginfo(f"Published /Measurements={self.x:.1f} at t={elapsed_time:.1f}")
        self.x+=0.5

if __name__ == '__main__':
    TimedPublisher()
    rospy.spin()

