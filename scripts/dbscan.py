#!/usr/bin/env python3

import rospy
import pandas as pd
from sklearn.cluster import DBSCAN
import seaborn as sns
import matplotlib.pyplot as plt
import time
import cv2


from sensor_msgs.msg import LaserScan
import math
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseArray

from std_msgs.msg import Header

#Maybe later we could use Pose stamped


# Global Variables
marker = Marker()
scan = LaserScan()


def fit_clusters_into_circles(clusters):

    fitted_circles = []

    for cluster in clusters:
        if len(cluster) < 3:
            # At least 3 points are needed to fit a circle
            continue

        # Convert the cluster from rectangular to polar coordinates
        # polar_cluster = []
        # for point in cluster:
        #     r = math.sqrt(point[0]**2 + point[1]**2)
        #     theta = math.atan2(point[1], point[0])
        #     polar_cluster.append([r, theta])

        # # Convert polar coordinates to numpy array
        polar_cluster = np.array(cluster,dtype=np.float32)

        # Fit a circle using minEnclosingCircle
        center, radius = cv2.minEnclosingCircle(polar_cluster)

        fitted_circles.append([center,radius])

    return fitted_circles


def polar_to_rectangular(Allclusters):
    rectangular_clusters = []
    
    for cluster in Allclusters:
        rectangular_points = []
        
        for point in cluster.reshape(-1,2):
            range_val, angle_val = point[0], point[1]
            x = range_val * np.cos(angle_val)
            y = range_val * np.sin(angle_val)
            rectangular_points.append([x, y])
        
        rectangular_clusters.append(rectangular_points)
    
    return rectangular_clusters

def polartorect(randtheta):
    return [randtheta[0] * math.cos(randtheta[1]), randtheta[0] * math.sin(randtheta[1])]

def polardistance(a, b):
    return math.sqrt(a[0]**2 + b[0]**2 - 2 * a[0] * b[0] * math.cos(a[1] - b[1]))


def visualization_point(center):

    #Object as sphere
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = 1

    marker.color.r = 0.5
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.pose.position.x=center[0]
    marker.pose.position.y=center[1]
    marker.pose.position.z=0

    visualpub.publish(marker)

    

def callback(msg):
    # Getting the polar coordinates of the point cloud
    global pose

    
    
    pts_r = np.array(msg.ranges)
    delfi = msg.angle_increment
    pts_ang = np.arange(start=msg.angle_min, stop=msg.angle_max, step=delfi)

    pts_r_list = list(pts_r)
    pts_ang_list = list(pts_ang)

    newscan_rect = []  #Contains point cloud in rectangular coordinates
    newscan_polar=[] #Contains point cloud in polar coordinates
    for r, ang in zip(pts_r_list, pts_ang_list):
        if ((not np.isinf(r)) and (abs(r)<2)):#Constraining scan to 1 radius circle 'and abs(r)<1'
            newscan_polar.append([r,ang])
            newscan_rect.append(polartorect([r, ang]))

    #We now have the scan in 1m radius. In polar as well as in rect.

    #Performing clustering


    df = pd.DataFrame(newscan_rect, columns =['x', 'y'])
    try:
        clustering = DBSCAN(eps=0.1, min_samples=3).fit(df)

    

        DBSCAN_dataset = df.copy()

        DBSCAN_dataset.loc[:,'Cluster'] = clustering.labels_ 

        DBSCAN_dataset.Cluster.value_counts().to_frame()

        outliers = DBSCAN_dataset[DBSCAN_dataset['Cluster']==-1]

        print("No. of clusters:")
        print(DBSCAN_dataset.Cluster.unique().size)
        cluster=DBSCAN_dataset[DBSCAN_dataset['Cluster']==0]
        
        clusters = []
        # Loop through clusters
        for label in DBSCAN_dataset['Cluster'].unique():
            if label == -1:
                # Skip outliers
                continue
            cluster = DBSCAN_dataset[DBSCAN_dataset['Cluster'] == label]
            clusters.append(cluster[['x', 'y']].values)

        print(cluster)

        #Fit clusters into circles
        try:
            fitted_circles = fit_clusters_into_circles(clusters) #fitcircle somehow seems to convert the coordiantes to positive
        except:
            print("There is error")

        people=[]
        for p in fitted_circles:
            print(p[1])
            if(p[1]<4 and p[1]>0.01):
                print("Person Detected!!")
                people.append(p)

        print(people)
        lidar_poses=PoseArray()

        lidar_poses.header = Header(stamp=rospy.Time.now(), frame_id="base_frame")

        
        for k in people:
            lidar_pose=Pose()
            lidar_pose.position.z,lidar_pose.position.x=list(k[0])

            lidar_poses.poses.append(lidar_pose)


        sns.scatterplot(x='x', y='y',

                data=DBSCAN_dataset[DBSCAN_dataset['Cluster']!=-1],

                hue='Cluster', palette='Set2', legend='full', s=10)
    
    

    
    
        plt.plot(0,0,'o')
    
        plt.xlim(-10,10)
        plt.ylim(-10,10)
    
        

        #print(cluster)
    
        plt.show(block=False)

        plt.pause(0.00000000001)

        plt.clf()

        #print(lidar_poses)

        

        # except:
        #     print("There is an error")



        # center_x=(sum(cluster['x'])/len(cluster['x']))
        # center_y=(sum(cluster['y'])/len(cluster['y']))

        # pose.position.x=center_x
        # pose.position.y=center_y

        # posepub.publish(pose)

        # center=[center_x,center_y]

        # print("Center1:")
        # print(center)

        #visualization_point(center)

    except:

        lidar_poses.poses=[]

        #pose_lidar_pub.publish(lidar_poses)

        # posepub.publish(pose)


    

    #print(DBSCAN_dataset)
    
    pose_lidar_pub.publish(lidar_poses)

    




def main():

    global visualpub,pose_lidar_pub#posepub,
    rospy.init_node('DBSCAN_Clustering')
    
    sub = rospy.Subscriber('/scan', LaserScan, callback)

    pose_lidar_pub=rospy.Publisher('/PoseLidar',PoseArray,queue_size=10)

    #posepub=rospy.Publisher('/Pose',Pose,queue_size=10)

    visualpub=rospy.Publisher('/visualpose',Marker,queue_size=10)

    '''while(not rospy.is_shutdown()):
        
        posepub.publish(pose)
        #visualpub.publish(marker)'''
    
    rospy.spin()
        
  

if __name__ == '__main__':
    main()
    






