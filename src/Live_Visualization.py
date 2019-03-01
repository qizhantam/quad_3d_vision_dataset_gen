#!/usr/bin/env python
import cv2, rospy
import Image_Processing
import Save_to_Coco_Format
from File_Operations import read_bag, save_image, closest_msg
from Coordinate_Transformations import vertex_coordinates
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


class Visualizer:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self.msg_pose_Quad = []
        self.msg_pose_RealSense = []
        self.cv_depth_Wireframe = Image_Processing.image_processing()
        self.cv_rgb_Wireframe   = Image_Processing.image_processing()

        rospy.Subscriber('/realsense/mavros/vision_pose/pose', PoseStamped, self.realsense_pose_callback)
        rospy.Subscriber('/quad_realsense/mavros/vision_pose/pose', PoseStamped, self.quad_pose_callback)
        # rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', PoseStamped, depth_callback)
        rospy.Subscriber('/camera/color/image_rect_color', Image, self.rgb_callback)
        rospy.spin()

    def realsense_pose_callback(self,msg):
        # print "hi2"
        self.msg_pose_RealSense = msg

    def quad_pose_callback(self,msg):
        self.msg_pose_Quad = msg

    def rgb_callback(self,msg):
        # just in case if realsense starts streaming before poses are received
        if (not self.msg_pose_Quad) or (not self.msg_pose_RealSense):
            return

        vertex_coords, max_coords, min_coords = vertex_coordinates(self.msg_pose_Quad,self.msg_pose_RealSense)

        ## Draw rectangle on images in opencv
        self.cv_rgb_Wireframe.convert_to_cv(msg)
        image_cv = self.cv_rgb_Wireframe.draw_wireframe(vertex_coords)
        # image_cv = self.cv_rgb_Wireframe.draw_rectangle([3,100],[0,0])
        cv2.imshow('image', image_cv)
        cv2.waitKey(1)


if __name__ == '__main__':
    vis = Visualizer()
