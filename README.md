# Project Setup and Execution

This guide outlines the steps required to run the perception stack using ROS. Follow the terminal instructions below in the given order to ensure proper execution.

## Prerequisites
- **ROS** installed
- Cloned this repository
- Necessary `.bag` files available(For simulation)

## Usage(For simulation)
### Step 1: Initialize ROS Core
Open **Terminal 1** and run:

```bash
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roscore
```

### Step 2: Start Image Processing Node
Open **Terminal 2** and run:

```bash
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
rosparam set use_sim_time true
ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
```

### Step 3: Launch AprilTag Ground Truth
Open **Terminal 4** and run:

```bash
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roslaunch apriltag_ros ground_truth.launch
```

### Step 4: Start Camera-LiDAR Projection
Open **Terminal 5** and run:

```bash
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roslaunch camera_2d_lidar_calibration reprojection.launch
```

### Step 5: Start Tracking 
Open **Terminal 6** and run:

```bash
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
roslaunch tracking tracking.launch
```

### Step 6: Play the Required Bag File
Open **Terminal 3** and run:

```bash
cd catkin_ws
source devel_isolated/apriltag_ros/setup.bash
cd <Bagfile location>
rosbag play <required_bagfile>
```

> **Note:** Replace `<Bagfile location>` with path of the bagfile directory and `<required_bagfile>` with your desired bag file name.


## Notes
- Ensure all dependencies are correctly sourced before launching nodes.


