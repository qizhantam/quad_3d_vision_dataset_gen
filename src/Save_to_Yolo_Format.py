
# Heavily modified from: https://github.com/hazirbas/coco-json-converter/blob/master/generate_coco_json.py

import os
import numpy as np
from os import walk
from File_Operations import copy_image, label_text
from ROSImageToCV2 import load_bag_date_timestamp

def Yolo_bbox_params(max_coords,min_coords,x_max_width,y_max_width):
    max_coords[0] = max_coords[0]/x_max_width
    max_coords[1] = max_coords[1]/y_max_width
    min_coords[0] = min_coords[0]/x_max_width
    min_coords[1] = min_coords[1]/y_max_width
    center_coords = [(max_coords[0]+min_coords[0])/2., (max_coords[1]+min_coords[1])/2.]
    width_coords  = [ max_coords[0]-min_coords[0]    ,  max_coords[1]-min_coords[1]    ]
    return center_coords, width_coords

def main():
    bag_date, bag_timestamp = load_bag_date_timestamp()
    save_image_Yolo_directory = '../export_data/' + bag_date + '/Yolo/' + bag_timestamp
    # save_label_text_directory = '../export_data/' + bag_date + '/Labels/' + bag_timestamp
    labels_directory          = '../export_data/' + bag_date + '/Labels/' + bag_timestamp
    unedited_directory  = '../export_data/' + bag_date + '/Unedited/' + bag_timestamp
    rectangle_directory = '../export_data/' + bag_date + '/drawn_Rectangle/' + bag_timestamp


    txt_name_list = []
    for (dirpath, dirnamese, filenames) in walk(rectangle_directory+"_rgb"):
        txt_name_list.extend(filenames)
        txt_name_list.sort(key=lambda f: int(filter(str.isdigit, f)))
        break

    max_coords = [0.,0.]
    min_coords = [0.,0.]
    for txt_name in txt_name_list:
        _ , file_postfix_png = txt_name.split("_")
        file_postfix , _     = file_postfix_png.split(".")
        rgb_file = unedited_directory + "_rgb/rgb_" + file_postfix_png
        depth_file = unedited_directory + "_depth/depth_" + file_postfix_png
        copy_image(rgb_file,save_image_Yolo_directory,"rgb",file_postfix_png)
        copy_image(depth_file,save_image_Yolo_directory,"depth",file_postfix_png)

        rgb_read = open(labels_directory + "_rgb/rgb_" + file_postfix + ".txt", "r")
        rgb_num = rgb_read.read().split(" ")
        max_coords[0] = float(rgb_num[1])
        max_coords[1] = float(rgb_num[2])
        min_coords[0] = float(rgb_num[3])
        min_coords[1] = float(rgb_num[4])
        center_coords, width_coords = Yolo_bbox_params(max_coords,min_coords,640.,480.)

        label_text(save_image_Yolo_directory, int(file_postfix), center_coords, width_coords, 'depth')
        label_text(save_image_Yolo_directory, int(file_postfix), center_coords, width_coords, 'rgb')

if __name__ == '__main__':
    main()
