# indoor_mobile_robot_perception

Implementaion:

Terminal 1:
  >>roslaunch tracking tracking_multiple.launch
  Description:
    Launches nodes for Clustering&Fitcircle, EKF and Visualising in rviz

Terminal 2:
cd bagfiles  
rosbag play recording2.bag --topics /scan  

  Description:
    Plays a recording of the lidar scan taken in the corrider with two people walking around

Expected output:

You ll be able to see the visualization of point cloud in rviz and the visualization of clustering in a matplot window.  
In the tracking_multiple.launch, you ll be able to see the number of people detected and their pose from EKF.  



