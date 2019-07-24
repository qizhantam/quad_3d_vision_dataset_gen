import os
import glob
import PIL
import PIL.Image as Image
import cv2
import numpy as np
from ROSImageToCV2 import load_bag_date_timestamp
from natsort import natsorted

# function to draw bounding box on the detected object with class name
def draw_bounding_box(img, confidence, x, y, x_plus_w, y_plus_h):
    label = "quad, " + str(confidence)
    color = (147,20,255)
    x = np.int32(float(x))
    y = np.int32(float(y))
    x_plus_w = np.int32(float(x_plus_w))
    y_plus_h = np.int32(float(y_plus_h))
    img = cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)
    img = cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return img

darknet_yolo_directory = '/home/qizhan/darknet' #TODO: Make sure to change this

bag_date, bag_timestamp = load_bag_date_timestamp()

current_directory = os.getcwd()

cfg_directory = './Yolo/cfg'

obj_data_file = cfg_directory + '/obj.data'
obj_data_file = os.path.abspath(obj_data_file)

yolov3_file = cfg_directory + '/yolov3_valid.cfg'
yolov3_file = os.path.abspath(yolov3_file)

weights_file = './Yolo/weights/yolov3_3700.weights'
weights_file = os.path.abspath(weights_file)

unedited_directory  = '../export_data/' + bag_date + '/Unedited/' + bag_timestamp
unedited_directory = os.path.abspath(unedited_directory)

results_directory = './Yolo/results'
results_directory = os.path.abspath(results_directory)

results_image = results_directory
results_file = results_directory + '/results_' + bag_timestamp + '.txt'
results_darknet_file = darknet_yolo_directory + '/results/comp4_det_test_' + 'Quad' + '.txt'

os.chdir(darknet_yolo_directory)
commands = './darknet detector valid ' + obj_data_file + ' ' + yolov3_file + ' ' + weights_file
os.system(commands)

results_out_of_order_func = open(results_darknet_file,'r')
results_out_of_order = results_out_of_order_func.readlines()
results_out_of_order_func.close()
results_in_order = natsorted(results_out_of_order)
results_func = open(results_file,'w')
for result in results_in_order:
    results_func.write(result)
results_func.close()

videoName = results_directory + '/' + bag_timestamp + '_video.mp4'
fourcc = cv2.VideoWriter_fourcc(*'X264')
video  = cv2.VideoWriter(videoName,fourcc,15.0,(640,480))

d = 0
image_List = []

image_filename_old = "initialize"
confidence_old = -1000
for line in results_in_order:
    (image_filename,confidence,min_x,min_y,max_x,max_y) = line.split(" ")
    print image_filename
    confidence = float(confidence)
    if (image_filename != image_filename_old) and (image_filename_old != "initialize"):
        cv2.imwrite(filename,img)
        video.write(img)

    if (image_filename != image_filename_old) or (confidence > confidence_old):
        # print unedited_directory + "_rgb/" + image_filename + ".png"
        img = cv2.imread(unedited_directory + "_rgb/" + image_filename + ".png")

        img = draw_bounding_box(img, confidence, min_x, min_y, max_x, max_y)
        filename = results_image + '/' + image_filename + '.png'
        cv2.imwrite(filename,img)
        image_filename_old = image_filename
        confidence_old = confidence

video.write(img)
video.release()
