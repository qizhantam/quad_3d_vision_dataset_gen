#!/usr/bin/env python
import math
import numpy as np
import os
import roslib
import sys
import rospy, rosbag
import cv2
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
import Image_Processing
import Save_to_Coco_Format

def read_bag(bag_date,bag_timestamp,bag_directory,bag_type):
    return rosbag.Bag(bag_directory + '/' + bag_type + '_' + bag_timestamp + '.bag')

def save_image(image,save_directory,bag_type,file_postfix):
    directory = save_directory + "_" + bag_type
    if not os.path.exists(directory):
      os.makedirs(directory)
    filename = directory + '/' + bag_type + ('_%i.png' % (file_postfix))
    cv2.imwrite(filename,image)
    return bag_type + ('_%i.png' % (file_postfix))

def save_image_for_YOLO(image,max_coords,min_coords,bag_date,bag_type,bag_timestamp,file_postfix):
    #WARNING: Work in progress...
    directory = '../export_data/' + bag_date + '/YOLO_dataset/' + bag_timestamp + bag_type

def closest_msg(t_compare,t_start,t_end,bag):
    #initialize with something ridiculous
    t_diff_old = rospy.Duration(-10000)
    t_old = rospy.Duration(-10000)

    #parse though the bag to find the message that matches most closely to input time
    for topic, msg, t in bag.read_messages(start_time=t_start,end_time=t_end):
        t_diff = t - t_compare
        if t_diff_old<=rospy.Duration(0) and t_diff>=rospy.Duration(0):
            if abs(t_diff_old)>abs(t_diff):
                continue
            else:
                msg = msg_old
                t = t_old
                continue
        t_old = t
        t_diff_old = t_diff
        msg_old = msg
    return msg, t

#Given poses --> return rectangle coordinates.
def vertex_coordinates(msg_pose_Quad,msg_pose_RealSense):
    #40x40x20cm
    half_length = 0.2 #m
    half_width = 0.2 #m
    half_height = 0.1 #m
    ori_vertex_array = np.array([[ half_length, half_width, half_height],
                                 [ half_length, half_width,-half_height],
                                 [ half_length,-half_width,-half_height],
                                 [ half_length,-half_width, half_height],
                                 [-half_length,-half_width, half_height],
                                 [-half_length,-half_width,-half_height],
                                 [-half_length, half_width,-half_height],
                                 [-half_length, half_width, half_height]])

    quaternion = (
        msg_pose_Quad.pose.orientation.x,
        msg_pose_Quad.pose.orientation.y,
        msg_pose_Quad.pose.orientation.z,
        msg_pose_Quad.pose.orientation.w)
    R_Quad = tf.transformations.quaternion_matrix(quaternion)
    R_Quad = R_Quad[:3,:3]
    C_Quad = np.array(
        [[msg_pose_Quad.pose.position.x],
         [msg_pose_Quad.pose.position.y],
         [msg_pose_Quad.pose.position.z]])

    quaternion = (
        msg_pose_RealSense.pose.orientation.x,
        msg_pose_RealSense.pose.orientation.y,
        msg_pose_RealSense.pose.orientation.z,
        msg_pose_RealSense.pose.orientation.w)
    R_camera = tf.transformations.quaternion_matrix(quaternion)
    R_camera = R_camera[:3,:3].T #inverse(transpose) rotation matrix
    C_camera = np.array(
        [[msg_pose_RealSense.pose.position.x],
         [msg_pose_RealSense.pose.position.y],
         [msg_pose_RealSense.pose.position.z]])
    t_camera = np.dot(R_camera,-C_camera)

    K_camera = np.array(
        [[617.2744140625, 0.0,              324.1011047363281],
         [0.0,            617.335693359375, 241.5790557861328],
         [0.0,            0.0,              1.0]]) #Intrinsic Parameters of Camera

    #Tuning
    R_delta = np.array([[1,0,0],
                        [0,0,1],
                        [0,-1,0]])
    C_delta = np.array([[0],
                        [0],
                        [0.]])
    t_delta = np.dot(R_delta,-C_delta)
    # R_delta = np.array([[ 0.99349809,  0.02414418,  0.11125917],
    #                     [-0.05171279, -0.77492165,  0.62993826],
    #                     [ 0.10142649, -0.63159599, -0.76863462]])
    # t_delta = np.array([[-1.45155382],
    #                     [-0.83107882],
    #                     [ 1.51691869]])

    vertex = np.zeros((3,1))
    vertex_pixels_array = np.zeros((8,2))
    for index in range(0,8):
        vertex[:3,0] = ori_vertex_array[index] #column vector, (x,y,z)
        vertex = np.dot(R_Quad,vertex)+C_Quad
        vertex_pixels = np.dot(K_camera,np.dot(R_delta,np.dot(R_camera,vertex)+t_camera)+t_delta)
        vertex_pixels = np.true_divide(vertex_pixels,vertex_pixels[2][0])
        vertex_pixels_array[index,:2] = vertex_pixels[:2,0]

    vertex_pixels_array = np.squeeze(vertex_pixels_array)
    max_coords = np.int32(np.amax(vertex_pixels_array,axis=0))
    min_coords = np.int32(np.amin(vertex_pixels_array,axis=0))
    return vertex_pixels_array, max_coords, min_coords

def main():
  file_postfix = 0

  #Update these for each separate recording
  bag_date = '20190206'
  bag_timestamp = '2019-02-06-22-42-24'

  bag_directory = '../record_data/' + bag_date
  bag_depth = read_bag(bag_date,bag_timestamp,bag_directory,'depth')
  bag_rgb = read_bag(bag_date,bag_timestamp,bag_directory,'rgb')
  bag_pose_RealSense = read_bag(bag_date,bag_timestamp,bag_directory,'pose_RealSense')
  bag_pose_Quad = read_bag(bag_date,bag_timestamp,bag_directory,'pose_Quad')

  # fourcc = cv2.VideoWriter_fourcc(*'X264')
  # videoName = '../export_data/' + bag_date + '/' + bag_timestamp + '_video.mp4'
  # video = cv2.VideoWriter(videoName,fourcc,30.0,(640,480))

  save_image_drawn_directory = '../export_data/' + bag_date + '/drawn/' + bag_timestamp
  save_image_Coco_directory = '../export_data/' + bag_date + '/Coco/' + bag_timestamp

  depth_annotation_Coco = Save_to_Coco_Format.CocoFormat(save_image_Coco_directory+"_depth")
  rgb_annotation_Coco = Save_to_Coco_Format.CocoFormat(save_image_Coco_directory+"_rgb")
  cv_depth = Image_Processing.image_processing()
  cv_rgb = Image_Processing.image_processing()
  for topic_depth, msg_depth, t_depth in bag_depth.read_messages():
      t_start = t_depth-rospy.Duration(0.1)
      t_end   = t_depth+rospy.Duration(0.1)
      ## Get messages with timestamps closest to the depth's message
      msg_rgb, t_rgb = closest_msg(t_depth,t_start,t_end,bag_rgb)
      msg_pose_RealSense, t_pose_RealSense = closest_msg(t_depth,t_start,t_end,bag_pose_RealSense)
      msg_pose_Quad, t_pose_Quad = closest_msg(t_depth,t_start,t_end,bag_pose_Quad)
      # print t_depth, (t_rgb-t_depth)/rospy.Duration(1), (t_pose_RealSense-t_depth)/rospy.Duration(1), (t_pose_Quad-t_depth)/rospy.Duration(1)

      ## Find projected pixel coordinates
      vertex_coords, max_coords, min_coords = vertex_coordinates(msg_pose_Quad,msg_pose_RealSense)


      # rect_coords = rectangle_coordinates(msg_pose_Quad,msg_pose_RealSense)
      # rect_top_left = (int(round(-100+rect_coords[0])),int(round(-100+rect_coords[1])))
      # rect_bot_right = (int(round(100+rect_coords[0])),int(round(100+rect_coords[1])))

      ## Draw rectangle on images in opencv
      cv_depth_image = cv_depth.convert_to_cv(msg_depth)
      cv_rgb_image = cv_rgb.convert_to_cv(msg_rgb)
      cv_depth_image_drawn = cv_depth.draw_rectangle(max_coords,min_coords)
      cv_rgb_image_drawn = cv_rgb.draw_rectangle(max_coords,min_coords)
      # cv_depth_image_drawn = cv.depth.draw_wireframe(max_coords,min_coords)
      # cv_rgb_image_drawn = cv.rgb.draw_wireframe(max_coords,min_coords)

      ## Save images
      depth_filename = save_image(cv_depth_image_drawn,save_image_drawn_directory,'depth',file_postfix)
      rgb_filename = save_image(cv_rgb_image_drawn,save_image_drawn_directory,'rgb',file_postfix)

      depth_filename = save_image(cv_depth_image,save_image_Coco_directory,'depth',file_postfix)
      rgb_filename = save_image(cv_rgb_image,save_image_Coco_directory,'rgb',file_postfix)
      depth_annotation_Coco.image_annotation(file_postfix, depth_filename, max_coords, min_coords)
      rgb_annotation_Coco.image_annotation(file_postfix, rgb_filename, max_coords, min_coords)
      # save_image_for_YOLO(cv_depth,max_coords,min_coords,bag_date,bag_timestamp,'depth',file_postfix)
      # save_image_for_YOLO(cv_rgb,max_coords,min_coords,bag_date,bag_timestamp,'rgb',file_postfix)

      # video.write(cv_rgb_image_drawn)
      file_postfix += 1

  depth_annotation_Coco.output_file()
  rgb_annotation_Coco.output_file()
  # cv2.destroyAllWindows()
  # video.release()

if __name__ == '__main__':
    main()
