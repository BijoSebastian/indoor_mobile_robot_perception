#!/usr/bin/env python3

# Import necessary libraries
import rospy
import pandas as pd
from sklearn.cluster import DBSCAN
import seaborn as sns
import matplotlib
matplotlib.use('TkAgg')  # Use the TkAgg backend (or another suitable backend)
import matplotlib.pyplot as plt
import time
from matplotlib.patches import Circle
import cv2
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseArray
from std_msgs.msg import Header
from scipy.optimize import least_squares
from tracking.msg import PoseID,PoseIDArray

#Maybe later we could use Pose stamped
# Global Variables
# background_scan = None
previous_scan = None
pose_lidar_pub = None
first_time = True
max_range = 6  # Set this to the maximum range of your lidar
near_tracking_range = 2
near_tracking_range_radius = 10
kalmanpositionarray = []
people_fitted_circles = []

# Global Variables
marker = Marker()
scan = LaserScan()
lidar_poses=PoseArray()
lidar_poses.poses=[]

def circle_residuals(params, x, y):
    # params contains [h, k, r] where (h, k) is the center and r is the radius
    # Calculate the residuals for the circle equation (x - h)^2 + (y - k)^2 = r^2
    h, k, r = params
    return np.sqrt((x - h)**2 + (y - k)**2) - r

def fit_circle(x, y):
    # Fit a circle to the points (x, y) using least squares optimization

    # Initial guess for the circle parameters
    h_guess = np.mean(x)  # Mean of x coordinates
    k_guess = np.mean(y)  # Mean of y coordinates
    r_guess = np.max(np.sqrt((x - h_guess)**2 + (y - k_guess)**2))  # Max distance from center
    initial_guess = [h_guess, k_guess, r_guess]
    result = least_squares(circle_residuals, initial_guess, args=(x, y))
    h, k, r = result.x
    return [h, k], r

def fit_clusters_into_circles2(clusters):

    # Fit clusters into circles using the fit_circle function
    # This function takes a list of clusters, where each cluster is a list of points in polar coordinates (range, angle)
    # It returns a list of fitted circles, where each circle is represented by its center and radius
    # Note: Here centre is not the centre of the fitted circle, but the mean of the x and y coordinates of the points in the cluster
    fitted_circles = []

    for cluster in clusters:
        if len(cluster) < 3:
            # At least 3 points are needed to fit a circle
            continue

        # # Convert polar coordinates to numpy array
        rect_cluster = np.array(cluster,dtype=np.float32)

        x=rect_cluster[:,0]
        y=rect_cluster[:,1]

        # Fit a circle using minEnclosingCircle
        center, radius = fit_circle(x,y)

        center = [np.mean(x),np.mean(y)]

        fitted_circles.append([center,radius])

    return fitted_circles

def polartorect(randtheta):
    # Convert polar coordinates (radius, angle) to rectangular coordinates (x, y)
    return [randtheta[0] * math.cos(randtheta[1]), randtheta[0] * math.sin(randtheta[1])]

def filter_infs(scan):
    # Filter out infinite values from the scan data
    # Replace infinite values with the maximum range
    return np.array([min(r, max_range) if not np.isinf(r) else max_range for r in scan])
 
def kalmancallback(pose_array):

    # Callback function to get position of already tracked people
    global kalmanpositionarray
    kalmanpositionarray = []
    for pose in pose_array.poses:
        kalmanpose = [pose.pose.position.x, pose.pose.position.y]
        kalmanpositionarray.append(kalmanpose)

def callback(msg):
    # time taken = 0.04s for callback to execute
    # Getting the polar coordinates of the point cloud
    global pose,first_time,t0,firstplottime,ax,fig,previous_scan,lidar_poses, kalmanpositionarray

    # Filter out infinite values from the scan data
    current_scan = filter_infs(np.array(msg.ranges))

    #Time handling
    if(first_time==True):
        # If this is the first scan, store it as the previous scan
        previous_scan = current_scan
        t0=msg.header.stamp.to_nsec()*(10**(-9))
        rospy.loginfo("Background scan stored")
        first_time=False
        return

    ptime_stamp=msg.header.stamp
    t_now=ptime_stamp.to_nsec()*(10**(-9))
    time_now=t_now-t0
    before_clustering_time=rospy.Time.now()
    before_clustering_time_sec=before_clustering_time.to_nsec()*(10**(-9))

    # Calculate the difference between the current scan and the previous scan
    difference = np.abs(current_scan - previous_scan)

    # Set a threshold for considering the difference as a significant change
    threshold = 0.01
    significant_indices = np.where(difference > threshold)[0]
    
    pts_r = current_scan[significant_indices]
    pts_ang = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))[significant_indices]

    newscan_rect = []  #Contains point cloud in rectangular coordinates
    newscan_polar=[] #Contains point cloud in polar coordinates
    # Filter out points that are too far away or infinite
    # and convert polar coordinates to rectangular coordinates
    # We are constraining the scan to a 1m radius circle
    for r, ang in zip(pts_r, pts_ang):
        if ((not np.isinf(r))) and (abs(r)<max_range):#Constraining scan to 1 radius circle 'and abs(r)<1'
            newscan_polar.append([r,ang])
            newscan_rect.append(polartorect([r, ang]))

    #We now have the scan in 1m radius. In polar as well as in rect.

    #Performing clustering


    df = pd.DataFrame(newscan_rect, columns =['x', 'y'])
    try:
        # Perform DBSCAN clustering
        # eps is the maximum distance between two samples for one to be considered as in the neighborhood
        # min_samples is the number of samples in a neighborhood for a point to be considered as a core point
        # Here, eps is set to 0.03 meters and min_samples is set to 3
        # You can adjust these parameters based on your specific use case
        clustering = DBSCAN(eps=0.03, min_samples=3).fit(df)

        DBSCAN_dataset = df.copy()

        DBSCAN_dataset.loc[:,'Cluster'] = clustering.labels_ 

        DBSCAN_dataset.Cluster.value_counts().to_frame()

        outliers = DBSCAN_dataset[DBSCAN_dataset['Cluster']==-1]

        cluster=DBSCAN_dataset[DBSCAN_dataset['Cluster']==0]
        
        clusters = []
        # Loop through clusters
        for label in DBSCAN_dataset['Cluster'].unique():
            if label == -1:
                # Skip outliers
                continue
            cluster = DBSCAN_dataset[DBSCAN_dataset['Cluster'] == label]
            clusters.append(cluster[['x', 'y']].values)

        #Fit clusters into circles
        try:
            fitted_circles = fit_clusters_into_circles2(clusters) #It ll say some serialization error
        except:
            print("!!!!There is error in fitting circle!!!!")

        people=[]
        people_fitted_circles = []
        for p in fitted_circles:
            center, radius = p
            near_tracked = False
            for kal_pos in kalmanpositionarray:
                # If the fitted circle's center is within 0.5m of a tracked person
                # Then the threshold for them to be considered as a person is increased
                if np.linalg.norm(np.array(center) - np.array(kal_pos)) < near_tracking_range:
                    near_tracked = True
                    break
            if near_tracked:
                # If the fitted circle's center is within 2m of a tracked person
                # then the threshold for them to be considered as a person is increased to 10m of radius
                if 0.01 < radius < near_tracking_range_radius:
                    people.append(center)
                    people_fitted_circles.append(p)
            else:
                # In general, if the fitted circle's center is within 0.2m of radius
                # then the threshold for them to be considered as a person is 0.2m
                if 0.01 < radius < 0.2:
                    people.append(center)
                    people_fitted_circles.append(p)

    
        # Person detections are published as a PoseArray
        # Each pose in the array corresponds to a detected person
        # The position of the pose is set to the average of the cluster of the fitted circle
        lidar_poses=PoseArray()

        lidar_poses.header = Header(stamp=ptime_stamp, frame_id="lidar")

        for k in people:
            lidar_pose=Pose()
            lidar_pose.position.x,lidar_pose.position.y=k

            lidar_poses.poses.append(lidar_pose)

        # ***************** Dynamic Plot Updating *****************
        # Create figure/axis only once.
        # if firstplottime:
        #     plt.ion()
        #     fig, ax = plt.subplots()
        #     firstplottime = False

        # # Clear the previous axis content.
        # ax.cla()
        
        # # Plot DBSCAN points.
        # # (Using standard matplotlib scatter for simplicity.)
        # # Use a color palette for distinct cluster colors
        # palette = sns.color_palette("husl", n_colors=len(DBSCAN_dataset['Cluster'].unique()))
        # cluster_labels = sorted(DBSCAN_dataset['Cluster'].unique())

        # for idx, label in enumerate(cluster_labels):
        #     cluster_points = DBSCAN_dataset[DBSCAN_dataset['Cluster'] == label]
        #     color = 'gray' if label == -1 else palette[idx]  # gray for outliers
        #     ax.scatter(cluster_points['x'], cluster_points['y'], s=20, color=color, label=f"Cluster {label}" if label != -1 else "Outliers")

        #     # Add cluster label text (skipping outliers)
        #     # if label != -1:
        #     #     x_mean = cluster_points['x'].mean()
        #     #     y_mean = cluster_points['y'].mean()
        #     #     ax.text(x_mean, y_mean, str(label), fontsize=10, color='black', ha='center', va='center',
        #     #             bbox=dict(facecolor='white', alpha=0.6, edgecolor='black', boxstyle='round,pad=0.2'))

        
        # ax.set_xlabel("X Position (meters)")
        # ax.set_ylabel("Y Position (meters)")
        # ax.plot(0, 0, 'o')  # Origin

        # # Draw the fitted circles (black outline).
        # for i in fitted_circles:
        #     circle = Circle(list(i[0]), i[1], fill=False, edgecolor='black')
        #     ax.add_patch(circle)

        # # Draw the circles that pass the threshold (blue outline).
        # for i in people_fitted_circles:
        #     center, radius = i
        #     circle = Circle(center, radius, fill=False, edgecolor='blue')
        #     ax.add_patch(circle)

        # # Draw circles for the kalman tracked positions (red outline, radius 1).
        # for i in kalmanpositionarray:
        #     circle = Circle(i, near_tracking_range, fill=False, color='red')
        #     ax.add_patch(circle)

        # # Fix the axes limits.
        # ax.set_xlim(-5, 5)
        # ax.set_ylim(-5, 5)
        
        # # Update the canvas.
        # fig.canvas.draw()
        # fig.canvas.flush_events()
        # plt.pause(0.001)
        # ***********************************************************
        #plt.close(fig)

    except Exception as error:

        lidar_poses.poses=[]
        print('!!!!There is error in clustering:',error)
        
    after_clustering_time=rospy.Time.now()
    after_clustering_time_sec=after_clustering_time.to_nsec()*(10**(-9))
    time_elapse=after_clustering_time_sec-before_clustering_time_sec
    
    previous_scan = current_scan
    
    pose_lidar_pub.publish(lidar_poses)

    


def main():

    global visualpub,pose_lidar_pub,first_time,firstplottime
    
    first_time=True
    firstplottime=True
    rospy.init_node('DBSCAN_Clustering')
    
    sub = rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
    sub2 = rospy.Subscriber('/kalmanposeArray', PoseIDArray, kalmancallback)

    pose_lidar_pub=rospy.Publisher('/PoseLidar',PoseArray,queue_size=1)
    
    rospy.spin()
        
  

if __name__ == '__main__':
    main()
    






