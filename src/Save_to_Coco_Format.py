## Heavily modified from: https://github.com/hazirbas/coco-json-converter/blob/master/generate_coco_json.py

import os
import cv2
import json
import numpy as np
from os import walk
from File_Operations import copy_image


class CocoFormat():
    def __init__(self, datapath):
        self.datapath = datapath
        self.info = {"year" : 2019,
                     "version" : " ",
                     "description" : " ",
                     "contributor" : " ",
                     "url" : " ",
                     "date_created" : "2019"
                    }
        self.licenses = [{"id": 1,
                          "name": " ",
                          "url": " "
                         }]
        self.type = "instances"
        self.categories = [{"id": 0, "name": "quad4", "supercategory": "quadrotor"}]
        self.images = []
        self.annotations = []

    def image_annotation(self, index_ID, filename, max_coords, min_coords):
        max_coords = np.array(max_coords,dtype=float)
        min_coords = np.array(min_coords,dtype=float)
        self.images.append({"date_captured" : "2016",
                       "file_name" : filename, # remove "/"
                       "id" : index_ID,
                       "license" : 1,
                       "url" : "",
                       "height" : 480,
                       "width" : 640})
        self.annotations.append({"segmentation" : [[]],
                            "area" : (max_coords[0]-min_coords[0])*(max_coords[1]-min_coords[1]),
                            "iscrowd" : 0,
                            "image_id" : index_ID,
                            "bbox" : [min_coords[0],min_coords[1],max_coords[0]-min_coords[0],max_coords[1]-min_coords[1]],
                            "category_id" : 0,
                            "id": index_ID})
        return self.images, self.annotations

    def output_file(self):
        json_data = {"info" : self.info,
                     "images" : self.images,
                     "licenses" : self.licenses,
                     "type" : self.type,
                     "annotations" : self.annotations,
                     "categories" : self.categories}

        with open(self.datapath+"/Coco_Annotations.json", "w") as jsonfile:
            json.dump(json_data, jsonfile, sort_keys=True, indent=4)

def main():
    bag_date = '20190206'
    bag_timestamp = '2019-02-06-22-42-24'
    save_image_Coco_directory = '../export_data/' + bag_date + '/Coco/' + bag_timestamp
    labels_directory          = '../export_data/' + bag_date + '/Labels/' + bag_timestamp
    unedited_directory  = '../export_data/' + bag_date + '/Unedited/' + bag_timestamp
    rectangle_directory = '../export_data/' + bag_date + '/drawn_Rectangle/' + bag_timestamp

    depth_annotation_Coco = CocoFormat(save_image_Coco_directory+"_depth")
    rgb_annotation_Coco   = CocoFormat(save_image_Coco_directory+"_rgb")

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
        copy_image(rgb_file,save_image_Coco_directory,"rgb",file_postfix_png)
        copy_image(depth_file,save_image_Coco_directory,"depth",file_postfix_png)

        rgb_read = open(labels_directory + "_rgb/rgb_" + file_postfix + ".txt", "r")
        rgb_num = rgb_read.read().split(" ")
        max_coords[0] = float(rgb_num[1])
        max_coords[1] = float(rgb_num[2])
        min_coords[0] = float(rgb_num[3])
        min_coords[1] = float(rgb_num[4])

        depth_annotation_Coco.image_annotation(txt_name, "depth_" + file_postfix, max_coords, min_coords)
        rgb_annotation_Coco.image_annotation(txt_name, "rgb_" + file_postfix, max_coords, min_coords)
    depth_annotation_Coco.output_file()
    rgb_annotation_Coco.output_file()

if __name__ == '__main__':
    main()
