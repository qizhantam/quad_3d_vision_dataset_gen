oInstalling dependencies
-----------------------
1. Install the Intel RealSense SDK and ROS wrapper: https://github.com/intel-ros/realsense
2. If using with the MSL/Stanford Flight Room set up, get ros_vrpn_client from https://github.com/StanfordMSL which also requires vrpn_catkin, glog_catkin, and catkin_simple ros packages.

Launching nodes to begin streaming data
---------------------------------------
These steps may be different for you depending on your installation configurations:
1. Terminal 1:
```
roscore
```
2. Terminal 2:
```
roslaunch realsense2_camera rs_rgbd.launch
```
3. Terminal 3:
	```
	roslaunch (fill in later for launching optitrack node)
	```
4. You can view the data-stream on RViz:

	```
	source /opt/ros/kinetic/setup.bash
	rosrun rviz rviz
	```
	After opening Rviz, set the correct frame: Rviz > Global Options > Fixed Frame > camera_link

	To start viewing the data-stream: Add > By Topic > [Select Ros Topic to view]

Saving Pose and Visual data from RealSense Camera and Optitrack
---------------------------------------------------------------
1. roslaunch vision_pose_estimation save_data.launch
	- save_data.launch:
		-	saves the pose of the camera & quad along with the the video streams of the depth & rgb into 4 separate rosbags. Make sure that the save directory is valid.
