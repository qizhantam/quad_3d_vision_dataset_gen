## Heavily modified from: https://github.com/hazirbas/coco-json-converter/blob/master/generate_coco_json.py

import os
import cv2
import json
import numpy as np


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
