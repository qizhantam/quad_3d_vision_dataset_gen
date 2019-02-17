import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_processing():
    def __init__(self):
        self.cv_image = CvBridge()

    def convert_to_cv(self,image_msg):
        self.image_cv = self.cv_image.imgmsg_to_cv2(image_msg)
        image = self.image_cv.copy()
        self.image_cv = image
        return self.image_cv

    #Given coordinates and image, draw rectangle on image.
    def draw_rectangle(self,max_coords, min_coords):
        min_coords = np.int32(min_coords)
        max_coords = np.int32(max_coords)
        rect_top_left = (min_coords[0],min_coords[1])
        rect_bot_right = (max_coords[0],max_coords[1])
        cv2.rectangle(self.image_cv,rect_top_left,rect_bot_right,(0,0,255),3)
        return self.image_cv

    def draw_wireframe(self,vertex_coords):
        cv_image = CvBridge()
        vertex_coords = np.int32(vertex_coords)
        cv2.polylines(self.image_cv,[vertex_coords[:4]],True,(0,0,255),3)
        cv2.polylines(self.image_cv,[vertex_coords[2:6]],True,(0,0,255),3)
        cv2.polylines(self.image_cv,[vertex_coords[4:8]],True,(0,0,255),3)
        temp_vertex_coords = np.append(vertex_coords[6:8],vertex_coords[:2],axis=0)
        cv2.polylines(self.image_cv,[temp_vertex_coords],True,(0,0,255),3)
        return self.image_cv
