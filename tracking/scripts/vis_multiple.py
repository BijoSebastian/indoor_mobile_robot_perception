#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Use the TkAgg backend (or another suitable backend)
import matplotlib.pyplot as plt
import math


from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Pose,PoseStamped,PoseArray
from nav_msgs.msg import Path
from tracking.msg import PoseID,PoseIDArray

#Object Initialization
kalman_path1=Path()
kalman_path2=Path()

measured_path1=Path()
measured_path2=Path()

apriltag_path1=Path()
apriltag_path2=Path()

measured_trajectories = {}
kalman_trajectories = {}
actual_trajectories = {}
predicted_trajectories = {}

# Define tolerance for time matching (e.g., 0.1 seconds)
TIME_TOLERANCE = 0.1



#Variable Initialization
#actualpath = []
kalmanpath = []

#Functions

'''def shutdown():

    #Prediction for next 10 steps:
    t=0
    while(t<20):
        prediction(X,P)
        kalmanpath.append([X[0][0], X[1][0]])
        t+=1


    #pathvis(actualpath, kalmanpath)'''

'''def pathvis(path1, path2):
    print("Actualpath:")
    print(path1)
    print("Kalmanpath:")
    print(path2)
    actual_x = [coord[0] for coord in path1]
    actual_y = [coord[1] for coord in path1]
    kalman_x = [coord[0] for coord in path2]
    kalman_y = [coord[1] for coord in path2]

    plt.xlabel('x')
    plt.ylabel('y')
    plt.scatter(0,0,color="black",marker="s")
    plt.plot(actual_x, actual_y,linewidth=4,color='red')
    plt.plot(kalman_x, kalman_y,linestyle="dashed",color='blue')
    plt.title('Actual path vs Kalman path')
    plt.savefig('path2.png')
    plt.show()
    #plt.waitforbuttonpress(0)

    image = cv2.imread('path2.png')
  
    Rotated_image = imutils.rotate(image, angle=90)

    cv2.imshow("Rotated", Rotated_image)

    cv2.waitKey(0)'''

def pose_to_position_april(pose):
    """
    Convert PoseArray message to a list of (x, y) positions.
    """
    #return (-pose.pose.position.z, -pose.pose.position.y) #Check why do I have to do this?
    return (pose.pose.position.x, pose.pose.position.y,pose.header.stamp.to_sec())
    #return (-pose.position.z, -pose.position.x)
    
def pose_to_position(pose):
    """
    Convert PoseArray message to a list of (x, y) positions.
    """
    return (pose.pose.position.x, pose.pose.position.y,pose.header.stamp.to_sec())

def update_trajectory(trajectory_dict, person_id, position):
    """
    Update trajectory of a specific person in the dictionary.
    """
    if person_id not in trajectory_dict:
        trajectory_dict[person_id] = []
    trajectory_dict[person_id].append(position)



def find_matching_times(times1, times2, tolerance):
    """
    Find the nearest timestamps within a certain tolerance.
    """
    matching_pairs = []

    for t1 in times1:
        min_diff = float('inf')
        best_match = None
        
        for t2 in times2:
            diff = abs(t1 - t2)
            
            if diff <= tolerance:
                if diff < min_diff:
                    min_diff = diff
                    best_match = t2

        if best_match is not None:
            matching_pairs.append((t1, best_match))

    return matching_pairs

# def plot_trajectories():
#     """
#     Plot trajectories of all persons.
#     """
#     color_cycle = plt.cm.tab10.colors
#     num_persons = len(measured_trajectories)
#     num_rows = math.ceil(num_persons / 2)  # Adjust the number of rows based on the number of persons
#     fig, axes = plt.subplots(num_rows, 2, figsize=(12, 6 * num_rows))  # Create subplots
#     axes = axes.flatten()
    
#     for idx, (person_id, measured_traj) in enumerate(measured_trajectories.items()):
#         kalman_traj = kalman_trajectories.get(person_id, [])
#         actual_traj = actual_trajectories.get(person_id, [])  # Use the correct key for actual trajectories

#         if len(kalman_traj) == 0:
#             continue

#         measured_color = color_cycle[idx % len(color_cycle)]  # Cycle through colors for different persons
#         kalman_color = color_cycle[(idx + 1) % len(color_cycle)]
#         actual_color = color_cycle[(idx + 2) % len(color_cycle)]
#         ax = axes[idx]
        
#         if measured_traj:
#             x, y = zip(*measured_traj)
#             ax.plot(x, y, label=f'Person {person_id} (Measured)', color=measured_color)
        
#         if kalman_traj:
#             x, y = zip(*kalman_traj)
#             ax.plot(x, y, label=f'Person {person_id} (Kalman)', color=kalman_color, linestyle='dashed')
        
#         if actual_traj:
#             x, y = zip(*actual_traj)
#             ax.plot(x, y, label=f'Person {person_id} (Actual)', color=actual_color, linestyle='dotted')

#         ax.set_xlabel('X')
#         ax.set_ylabel('Y')
#         ax.set_title(f'Trajectories of Person {person_id}')
#         ax.legend()
#         ax.grid(True)
        
#         # Set axis limits to always range from -3 to +3
#         ax.set_xlim([-3, 3])
#         ax.set_ylim([-3, 3])
#         ax.plot(0, 0, 'o')  # Mark the origin

#     plt.tight_layout()
#     plt.show()
#     plt.pause(10)

def plot_trajectories():
    """
    Plot trajectories of all persons.
    """
    #plt.figure()
    color_cycle = plt.cm.tab10.colors
    num_persons = len(measured_trajectories)
    num_rows = math.ceil(num_persons / 2)  # Adjust the number of rows based on the number of persons
    fig, axes = plt.subplots(num_rows+1, 2, figsize=(12, 6 * (num_rows+1)))  # Create subplots
    axes = axes.flatten()

    mse_measured_actual = {}
    mse_kalman_actual = {}
    mse_predicted_actual = []
    time_measured_actual = []
    time_kalman_actual = []

    def calculate_error_over_time(traj1, traj2):
            times1 = [t for _, _, t in traj1]
            times2 = [t for _, _, t in traj2]
            matching_pairs = find_matching_times(times1, times2, TIME_TOLERANCE)
            if not matching_pairs:
                return [], []

            traj1_dict = {t: (x, y) for x, y, t in traj1}
            traj2_dict = {t: (x, y) for x, y, t in traj2}
            errors = [(traj1_dict[t1][0] - traj2_dict[t2][0])**2 + (traj1_dict[t1][1] - traj2_dict[t2][1])**2
                      for t1, t2 in matching_pairs]
            return errors, [t1 for t1, _ in matching_pairs]

    #for person_id, measured_traj in measured_trajectories.items():
    for idx, (person_id, measured_traj) in enumerate(measured_trajectories.items()):
        
        kalman_traj = kalman_trajectories.get(person_id, [])
        actual_traj = actual_trajectories.get(person_id, []) 
        predicted_traj = predicted_trajectories.get(person_id, [])

        if not actual_traj:
            print("The trajectory list is empty. Cannot plot trajectories.")
            continue
        if(len(kalman_traj)==0 and len(predicted_traj)==0):
            continue

        measured_color = color_cycle[idx % len(color_cycle)]  # Cycle through colors for different persons
        kalman_color = color_cycle[(idx + 1) % len(color_cycle)]
        actual_color = color_cycle[(idx + 2) % len(color_cycle)]
        predicted_color = color_cycle[(idx + 3) % len(color_cycle)]
        ax = axes[idx]

        if measured_traj:
            measured_x, measured_y, measured_time = zip(*measured_traj)
            ax.plot(measured_x, measured_y, label=f'Person {person_id} (Measured)', color=measured_color)
        if kalman_traj:
            kalman_x, kalman_y, kalman_time = zip(*kalman_traj)
            ax.plot(kalman_x, kalman_y, label=f'Person {person_id} (Kalman)', color=kalman_color, linestyle='dashed')
        if actual_traj:    
            actual_x, actual_y, actual_time = zip(*actual_traj)
            ax.plot(actual_x, actual_y, label=f'Person {person_id} (Actual)', color=actual_color, linestyle='dotted')
        if predicted_traj:
            predicted_x, predicted_y, predicted_time = zip(*predicted_traj)
            ax.plot(predicted_x, predicted_y, label=f'Person {person_id} (Predicted)', color=predicted_color, linestyle='dashdot')
        
        #plt.plot(x, y, label=f'Person {person_id} (Measured)', color='blue')
        ax.set_xlabel('X(metres)')
        ax.set_ylabel('Y(metres)')
        ax.set_title(f'Trajectories of Person {person_id}')
        ax.legend()
        ax.grid(True)
        #plt.plot(x, y, label=f'Person {person_id} (Kalman)', color='red', linestyle='dashed')
        ax.plot(0,0,'o')

        # Calculate MSE
        # def calculate_mse(traj1, traj2):
        #     times1 = [t for _, _, t in traj1]
        #     times2 = [t for _, _, t in traj2]
        #     common_times = set(times1) & set(times2)
        #     if not common_times:
        #         return float('inf')

        #     traj1_dict = {t: (x, y) for x, y, t in traj1}
        #     traj2_dict = {t: (x, y) for x, y, t in traj2}
        #     mse = np.mean([(traj1_dict[t][0] - traj2_dict[t][0])**2 + (traj1_dict[t][1] - traj2_dict[t][1])**2 for t in common_times])
        #     return mse

        mse_measured_actual[person_id], _ = calculate_error_over_time(measured_traj, actual_traj)
        mse_kalman_actual[person_id], _ = calculate_error_over_time(kalman_traj, actual_traj)
        print('person id:',person_id)
        print('kalman_traj:',kalman_traj)

        # Plotting error over time for each person
    ax_error_measured = axes[-2]  # Use the second last subplot for measured errors
    ax_error_kalman = axes[-1]  # Use the last subplot for Kalman errors
        
        # measured_errors, measured_times = calculate_error_over_time(measured_traj, actual_traj)
        # kalman_errors, kalman_times = calculate_error_over_time(kalman_traj, actual_traj)
        # print('measured_traj:',measured_traj)
    

    # Plotting error over time for each person
    print('mse_kalman_actual:',mse_kalman_actual)

    colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']

    for person_id, errors in mse_measured_actual.items():
        if errors:
            times = [i for i in range(len(errors))]
            
            ax_error_measured.plot(times, errors, label=f'Person {person_id} (Measured vs Actual)', linestyle='dashed')

            max_error = np.max(errors)
            min_error = np.min(errors)
            mean_error = np.mean(errors)
            # ax_error_measured.axhline(y=max_error, color='red', linestyle='--', label=f'Max Error({person_id})')
            # ax_error_measured.axhline(y=min_error, color='green', linestyle='--', label=f'Min Error({person_id})')
            color = colors[person_id % len(colors)]

            ax_error_measured.axhline(y=mean_error, color=color, linestyle='--', label=f'Mean Error({person_id})')
            print('mean_error:',mean_error)

    for person_id, errors in mse_kalman_actual.items():
        if errors:
            times = [i for i in range(len(errors))]
            ax_error_kalman.plot(times, errors, label=f'Person {person_id} (Kalman vs Actual)', linestyle='dashed')

            max_error = np.max(errors)
            min_error = np.min(errors)
            mean_error = np.mean(errors)
            # ax_error_kalman.axhline(y=max_error, color='red', linestyle='--', label=f'Max Error({person_id})')
            # ax_error_kalman.axhline(y=min_error, color='green', linestyle='--', label=f'Min Error({person_id})')
            color = colors[person_id % len(colors)]
            
            ax_error_kalman.axhline(y=mean_error, color=color, linestyle='--', label=f'Mean Error({person_id})')
            print('mean_error:',mean_error)

    ax_error_measured.set_xlabel('Time (indices)')
    ax_error_measured.set_ylabel('Mean Square Error')
    ax_error_measured.set_title('Error Over Time (Measured vs Actual)')
    ax_error_measured.legend(framealpha=0.1)
    ax_error_measured.grid(True)

    ax_error_kalman.set_xlabel('Time (indices)')
    ax_error_kalman.set_ylabel('Mean Square Error')
    ax_error_kalman.set_title('Error Over Time (Kalman vs Actual)')
    ax_error_kalman.legend(framealpha=0.1)
    ax_error_kalman.grid(True)

    plt.tight_layout()
    plt.show()
    plt.pause(10)

    print(f'MSE (Measured vs Actual): {mse_measured_actual}')
    print(f'MSE (Kalman vs Actual): {mse_kalman_actual}')

    #     mse_measured_actual.extend(measured_errors)
    #     mse_kalman_actual.extend(kalman_errors)
    #     time_measured_actual.extend(measured_times)
    #     time_kalman_actual.extend(kalman_times)

    #         # Plotting error over time as a subplot
    # error_ax_measured = axes[-2]  # Use the second last subplot for measured errors
    # error_ax_kalman = axes[-1]  # Use the last subplot for Kalman errors

    # print('Length of time_measured_actual:',len(time_measured_actual))
    # print('Length of mse_measured_actual:',len(mse_measured_actual))

    # if time_measured_actual and mse_measured_actual:
    #     error_ax_measured.plot(time_measured_actual, mse_measured_actual, label='Error (Measured vs Actual)', color='green')
    #     error_ax_measured.set_xlabel('Time (seconds)')
    #     error_ax_measured.set_ylabel('Mean Square Error')
    #     error_ax_measured.set_title('Error Over Time (Measured vs Actual)')
    #     error_ax_measured.legend()
    #     error_ax_measured.grid(True)

    # print('Length of time_kalman_actual:',len(time_kalman_actual))
    # print('Length of mse_kalman_actual:',len(mse_kalman_actual))

    # if time_kalman_actual and mse_kalman_actual:
    #     print('Plotting error vs time')
    #     error_ax_kalman.plot(time_kalman_actual, mse_kalman_actual, label='Error (Kalman vs Actual)', color='purple')
    #     error_ax_kalman.set_xlabel('Time (seconds)')
    #     error_ax_kalman.set_ylabel('Mean Square Error')
    #     error_ax_kalman.set_title('Error Over Time (Kalman vs Actual)')
    #     error_ax_kalman.legend()
    #     error_ax_kalman.grid(True)
        
    #     # mse_measured_actual.append(calculate_mse(measured_traj, actual_traj))
    #     # mse_kalman_actual.append(calculate_mse(kalman_traj, actual_traj))
    #     # mse_predicted_actual.append(calculate_mse(predicted_traj, actual_traj))


    # # print(f'MSE (Measured vs Actual): {np.mean(mse_measured_actual)}')
    # # print(f'MSE (Kalman vs Actual): {np.mean(mse_kalman_actual)}')
    # # print(f'MSE (Predicted vs Actual): {np.mean(mse_predicted_actual)}')

    # plt.tight_layout()
    # # plt.xlabel('X')
    # # plt.ylabel('Y')
    # # plt.title('Trajectories of Persons')
    # # plt.legend()
    # #plt.grid(True)
    # #plt.savefig('person_path.png')
    
    # plt.show()
    # plt.pause(10)

    # # plt.pause(0.001)
    # # plt.clf()
    # # plt.close()
    # #plt.show()

    # # Plotting error over time
    # # plt.figure(figsize=(12, 6))
    # # if time_measured_actual and mse_measured_actual:
    # #     plt.plot(time_measured_actual, mse_measured_actual, label='Error (Measured vs Actual)', color='green')
    # # if time_kalman_actual and mse_kalman_actual:
    # #     plt.plot(time_kalman_actual, mse_kalman_actual, label='Error (Kalman vs Actual)', color='purple')

    # # plt.xlabel('Time (seconds)')
    # # plt.ylabel('Mean Square Error')
    # # plt.title('Error Over Time')
    # # plt.legend()
    # # plt.grid(True)
    # # plt.show()

    # print(f'MSE (Measured vs Actual): {np.mean(mse_measured_actual)}')
    # print(f'MSE (Kalman vs Actual): {np.mean(mse_kalman_actual)}')

    # # print(f'MSE (Measured vs Actual): {np.mean(mse_measured_actual)}')
    # # print(f'MSE (Kalman vs Actual): {np.mean(mse_kalman_actual)}')
    # # print(f'MSE (Predicted vs Actual): {np.mean(mse_predicted_actual)}')

def visualization_markers(posearray,publisher):

    #Object as sphere
    markerarray=MarkerArray()

    tempmarker=Marker()
    
    tempmarker.header.frame_id = "map"
    tempmarker.type = Marker.SPHERE

    tempmarker.scale.x = 0.1
    tempmarker.scale.y = 0.1
    tempmarker.scale.z = 0.1

    for i in posearray.poses:
        
        tempmarker.header.stamp = posearray.header.stamp
        
        tempmarker.action = Marker.ADD
        tempmarker.id = i.ID
        tempmarker.color.r = i.ID/10
        tempmarker.color.g = 1-(i.ID/10)   #It ll saturate if no. of people being tracked is more than 10
        tempmarker.color.b = 0.5
        tempmarker.color.a = 1.0

        tempmarker.pose.position.x=i.pose.position.x
        tempmarker.pose.position.y=i.pose.position.y
        tempmarker.pose.position.z=0

        markerarray.markers.append(tempmarker)

    publisher.publish(markerarray)
    

def kalman_callback(pose_array):
    visualization_markers(pose_array,kalman_markerarray_pub)
    try:
        pose_array.poses[0].header.stamp=pose_array.header.stamp
    except Exception as err:
        print("ERROR in visualization pose array header assignment:")
        print(err)
    for pose in pose_array.poses:
        position = pose_to_position(pose)
        update_trajectory(kalman_trajectories, pose.ID, position)
    #plot_trajectories()
    #kalman_pathmaker(pose_array.poses[0],kalman_path1,kalman_path_pub1)
    #kalman_pathmaker(pose_array.poses[1],kalman_path2,kalman_path_pub2)

# def kalman_pathmaker(pose,kalman_path,kalman_path_pub):

#     x = pose.pose.position.x
#     y = pose.pose.position.y
          
#     kalman_path.header.frame_id="map"
#     kalman_path.header.stamp=pose.header.stamp
#     pose_current = PoseStamped()
#     pose_current.pose.position.x = x
#     pose_current.pose.position.y = y
#     pose_current.pose.position.z = 0
#     kalman_path.poses.append(pose_current)

#     kalman_path_pub.publish(kalman_path)

def predicted_callback(pose_array):
    visualization_markers(pose_array, predicted_markerarray_pub)
    for pose in pose_array.poses:
        person_id = pose.ID
        position = pose_to_position(pose)
        update_trajectory(predicted_trajectories, person_id, position)
    # Optionally, you can call plot_trajectories() here to update the plots in real-time
    # plot_trajectories()


def measured_callback(pose_array):
    # measured_pathmaker(pose_array.poses[0],measured_path1,measured_path_pub1)
    # measured_pathmaker(pose_array.poses[1],measured_path2,measured_path_pub2)
    visualization_markers(pose_array,measured_markerarray_pub)
    try:
        pose_array.poses[0].header.stamp=pose_array.header.stamp
    except Exception as err:
        print("ERROR in visualization pose array header assignment:")
        print(err)
    for pose in pose_array.poses:
        position = pose_to_position(pose)
        update_trajectory(measured_trajectories, pose.ID, position)
    #plot_trajectories()

def apriltag_callback(pose_array):
    #visualization_markers(pose_array,apriltag_markerarray_pub)
    for pose in pose_array.poses:
        #Assign Apriltag id as person id
        person_id = pose.ID
        #print(f'$$$$Person id:{person_id}$$$$')
        #print('person id:',person_id)
        # if person_id == 12: #FOR 2 PEOPLE ROSBAG
        #     person_id=2
        #     #print('Person id is 13')
        # if person_id == 13:
        #     person_id=3

        if person_id == 13: #FOR 1 PEOPLE ROSBAG
            person_id = 3 #changed for multiple4
            #print('Person id is 13')
        if person_id == 12:
            person_id=2

        position = pose_to_position_april(pose)
        update_trajectory(actual_trajectories, person_id, position)
        #print('UPDATING!')


def measured_pathmaker(pose,measured_path,measured_path_pub):

    x = pose.position.x
    y = pose.position.y

    if(not math.isnan(x) and not math.isnan(y)):  
        measured_path.header.frame_id="map"
        measured_path.header.stamp=rospy.Time.now()
        pose_current = PoseStamped()
        pose_current.pose.position.x = x
        pose_current.pose.position.y = y
        pose_current.pose.position.z = 0
        measured_path.poses.append(pose_current)

        measured_path_pub.publish(measured_path)



def main():
    global kalman_path_pub1,kalman_path_pub2,measured_path_pub1,measured_path_pub2,measured_markerarray_pub,kalman_markerarray_pub,predicted_markerarray_pub
    rospy.init_node('Visualisation_Node')
    print('Visualisation started')

    kalman_pose_sub = rospy.Subscriber('/kalmanposeArray',PoseIDArray,kalman_callback)

    measured_pose_sub = rospy.Subscriber('/Measurements',PoseIDArray,measured_callback)

    apriltag_pose_sub = rospy.Subscriber('/ground_truth',PoseIDArray,apriltag_callback)

    predicted_pose_sub = rospy.Subscriber('/PredictedPoses', PoseIDArray, predicted_callback) 

    apriltag_path_pub1=rospy.Publisher('/apriltag_path1',Path,queue_size=20)
    apriltag_path_pub2=rospy.Publisher('/apriltag_path2',Path,queue_size=20)    

    apriltag_markerarray_pub=rospy.Publisher('/apriltag_markers',MarkerArray,queue_size=20)
    kalman_path_pub1=rospy.Publisher('/kalman_path1',Path,queue_size=20)
    kalman_path_pub2=rospy.Publisher('/kalman_path2',Path,queue_size=20)

    measured_path_pub1=rospy.Publisher('/measured_path1',Path,queue_size=20)
    measured_path_pub2=rospy.Publisher('/measured_path2',Path,queue_size=20)

    measured_markerarray_pub=rospy.Publisher('/measured_markers',MarkerArray,queue_size=20)
    kalman_markerarray_pub=rospy.Publisher('/kalman_markers',MarkerArray,queue_size=20)
    predicted_markerarray_pub = rospy.Publisher('/predicted_markers', MarkerArray, queue_size=20)  # New publisher

    rospy.on_shutdown(plot_trajectories)

    #rospy.on_shutdown(shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()

#The nnumber of circles keeps changing