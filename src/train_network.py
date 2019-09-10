import os
import glob
import PIL
import PIL.Image as Image
import cv2
import numpy as np
from ROSImageToCV2 import load_bag_date_timestamp
from natsort import natsorted

##############################################
## These must be set
base_path = '/home/awc11/Documents/quad_dataset_gen'
darknet_yolo_directory = '/home/awc11/Documents/darknet'
b_use_pretrained_weights = False
##############################################


# bag_date, bag_timestamp = load_bag_date_timestamp()

current_directory = os.getcwd()

cfg_directory = base_path + '/src/Yolo/cfg'

obj_data_file = cfg_directory + '/obj.data'
obj_data_file = os.path.abspath(obj_data_file)

yolov3_file = cfg_directory + '/yolov3_valid.cfg'
yolov3_file = os.path.abspath(yolov3_file)

# weights_file = base_path + '/src/Yolo/weights/yolov3_3700.weights'
weights_file = base_path + '/src/Yolo/weights/yolov3_dronenet.weights'
weights_file = os.path.abspath(weights_file)


# Pretrained Weights (OPTIONAL)
if b_use_pretrained_weights:
	pretrained_weight_file = base_path + '/src/Yolo/weights/yolov3_3700.weights'
	pretrained_weight_file = os.path.abspath(pretrained_weight_file)



os.chdir(darknet_yolo_directory)
if b_use_pretrained_weights:
	commands_pretraining = './darknet partial' + obj_data_file + ' ' + yolov3_file + ' ' + weights_file
	# ./darknet partial cfg/darknet19_448.cfg darknet19_448.weights darknet19_448.conv.23 23
	os.system(commands_pretraining)
commands = './darknet detector train ' + obj_data_file + ' ' + yolov3_file + ' ' + weights_file
os.system(commands)
