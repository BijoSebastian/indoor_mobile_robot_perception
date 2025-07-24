# Project Setup and Execution

This guide outlines the steps required to run the perception stack using ROS. Follow the terminal instructions below in the given order to ensure proper execution.

## Prerequisites
- **ROS** installed
- Cloned this repository
- Necessary `.bag` files available(For simulation)

## Usage
### Step 1: Launch your robot and sensors
Open **Terminal 1** and run:

```bash
roslaunch rplidar_ros rplidar_a1.launch
```

Open **Terminal 2** and run:

```bash
roslaunch usb_cam usb_cam-test.launch
```

Also, launch required nodes to get position of robot topic (in case your using Indoor mobile robot - /pose_ekf)

### Step 2: Start Image Processing Node
Open **Terminal 3** and run:

```bash
ROS_NAMESPACE=usb_cam rosrun image_proc image_proc
```



### Step 3: Start Camera-LiDAR Projection
Open **Terminal 4** and run:

```bash
roslaunch camera_2d_lidar_calibration reprojection_real.launch
```

### Step 4: Start Tracking 
Open **Terminal 5** and run:

```bash
roslaunch tracking tracking_real.launch
```

### Step (For simulation) : Play the Required Bag File (Instead of step 1)
Open **Terminal 1** and run:

```bash
cd <Bagfile location>
rosbag play <required_bagfile>
```

### Step (For Ground truth) : Launch AprilTag Ground Truth ( OPTIONAL )
Open **Terminal 6** and run:

```bash
cd catkin_ws
roslaunch apriltag_ros ground_truth.launch
```

> **Note:** Replace `<Bagfile location>` with path of the bagfile directory and `<required_bagfile>` with your desired bag file name.


## Notes
- Ensure all dependencies are correctly sourced before launching nodes.


