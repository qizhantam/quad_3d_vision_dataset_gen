# quad_dataset_gen
Generating labeled 3D vision dataset for quadrotor pose detection. This repository contains the revamped code used for the first half of my Winter 2019's AA290 project with a ton of help from Eric Cristofalo and Zijian Wang. The project report [AA290_WinterReport.pdf](AA290_WinterReport.pdf) is included and I hope it will help describe the application of this repository. Please don't hesitate to contact me for any questions or bugs.

## Sections
1. [Installing Dependencies](#installing-dependencies)
2. [Recording Data](#recording-data)
3. [Data Annotation](#data-annotation)
4. [Training on YOLO](#training-on-yolo)
5. [Tracking a Drone](#tracking-a-drone)

## Installing dependencies
1. Install the Intel RealSense SDK and ROS wrapper: https://github.com/intel-ros/realsense
2. If using with the MSL/Stanford Flight Room set up, get ros_vrpn_client from https://github.com/StanfordMSL which also requires vrpn_catkin, glog_catkin, and catkin_simple ros packages.
3. Yolo: https://pjreddie.com/darknet/yolo/

## Recording Data
### Launching nodes to begin streaming data
These steps may be different for you depending on your installation configurations:
1. Terminal 1:
	```
	roscore
	```
2. Terminal 2:
	```
	roslaunch realsense2_camera rs_rgbd.launch
	```
3. Copy vrpn_quad_realsense.launch and vrpn_realsense.launch to the ros_vrpn_client launch folder. Note: Make sure that the rostopics are correct, "vrpn_name" corresponds to the asset name in Motive Tracker and "robot_ns" is the same name as the ros nodes to be saved.

	Terminals 3 & 4:
	```
	roslaunch ros_vrpn_client vrpn_quad_realsense.launch
	roslaunch ros_vrpn_client vrpn_realsense.launch
	```
4. (Optional) You can view the data-stream on RViz:

	```
	source /opt/ros/kinetic/setup.bash
	rosrun rviz rviz
	```
	- After opening Rviz, set the correct frame: Rviz > Global Options > Fixed Frame > camera_link
	- To start viewing the data-stream: Add > By Topic > [Select Ros Topic to view]

5. (Optional, for calibration/debugging of transformation matrices.)

 To view the drawn wireframe bounding box:
	```
	rosrun quad_dataset_gen Live_Visualization.py
	```

### Saving Pose and Visual data from RealSense Camera and Optitrack
1. Save the pose of the camera & quad along with the the video streams of the depth & rgb into 4 separate rosbags (Make sure that the save directory/folder is created beforehand and the rostopics are correct.):

	```
	roslaunch quad_dataset_gen save_data.launch
	```
For example, when launched, the default path where the rosbags will be saved is: /home/qizhan/catkin_ws/src/quad_dataset_gen/record_data/20190309.

## Data Annotation for Training
### Bounding box verification
1. Export the ros rgb messages to OpenCV format and draw the bounding boxes using the recorded pose data of the camera and quad. In ROSImageToSC2.py, under the function load_bag_date_timestamp, check that bag_date is the folder where the data is saved, and bag_timestamp is the timestamp appended to the end of the bag files.

	```
	python ROSImageToCV2.py
	```
	The default settings exports frames with and without rectangular bounding boxes along with the coordinates to the /export_data directory. You can also output videos, draw 3D bounding boxes ("Wireframe"), output Coco annotated files by changing the settings.

2. Go to the directory where the drawn rgb images are located (export_data/20190309/drawn_Rectangle/2019-03-09-12-19-21_rgb/) and delete any images that have poor bounding boxes drawn. This will remove the images from the next step.

### Export annotation for YOLO training
1. The verified images and their label text files can then be saved to a folder "Yolo" in the /export_data directory:
	```
	python Save_to_Yolo_Format.py
	```
2. Adjust the train to test ratio in Yolo_Train_Test_List.py. Running the script will create train.txt and test.txt files in the directory /src/Yolo/:
	```
	python Yolo_Train_Test_List.py
	```

## Training on Yolo
0. Note: As the untrained weights "darknet53.conv.74" and trained weights "yolov3_3700.weights" exceed github file size limits, I've uploaded them to the MSL google drive. Download these 2 files and put them in /src/Yolo/weights/. Make sure to change the directories in /src/Yolo/cfg/obj.data.

1. This will require installation of Yolo. As an example, in the darknet folder (where I installed Yolo), I would type:
	```
	./darknet detector train /home/qizhan/catkin_ws/src/quad_dataset_gen/src/Yolo/cfg/obj.data /home/qizhan/catkin_ws/src/quad_dataset_gen/src/Yolo/cfg/yolov3_train.cfg /home/qizhan/catkin_ws/src/quad_dataset_gen/src/Yolo/weights/darknet53.conv.74
	```
	cfg.data and yolov3.cfg are modified files from the darknet/Yolo folder. You can find the files and more under /quad_dataset_gen/src/Yolo/.

2. As only a small sample of the dataset is included with this repository, the trained weights on the quad dataset is included in /quad_dataset_gen/src/Yolo/ as yolov3_3700.weights.

## Tracking a Drone (post-processing)
0. Note: "yolov3_3700.weights" is required to be in /src/Yolo/weights/, see [Training on YOLO](#training-on-yolo). Make sure to change the directories in /src/Yolo/cfg/obj.data.

1. Going back to quad_dataset_gen/src/, we do a forward pass on all the images using the trained weights and save the detected bounding boxes and a video of them in /src/Yolo/results/:
	```
	python Save_Yolo_Ouput_Images.py
	```
	*Check or modify the directory paths and commands specified in the script, especially what goes into "darknet_yolo_directory".

2. Now we can finally run the detected bounding boxes through an EKF to see how well we can track the quad's pose.
	```
	python Tracking.py
	```
	When it's done running, a plot window should pop-up displaying the comparison between the ground truth and the EKF's predictions. A video of the quad with the EKF's predicted bounding box drawn over it should also be automatically saved in the export folder. There are also lines in Tracking.py that you can uncomment to record the animation of the plot in real time.
