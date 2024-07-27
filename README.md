List of commands to execute:

Terminal 1:
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roscore

Terminal 2:
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
rosparam set use_sim_time true
ROS_NAMESPACE=usb_cam rosrun image_proc image_proc

Terminal 3:
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
cd src/bagfiles/final_tests/
rosbag play <-required_bagfile->

Terminal 4:
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roslaunch apriltag_ros ground_truth.launch

Terminal 5:
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roslaunch camera_2d_lidar_calibration reprojection.launch


Terminal 6:
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roslaunch tracking tracking.launch
