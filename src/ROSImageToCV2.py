#!/usr/bin/env python
import cv2, rospy
import Image_Processing
import Save_to_Coco_Format
from File_Operations import read_bag, save_image, closest_msg
from Coordinate_Transformations import vertex_coordinates

def save_image_for_YOLO(image,max_coords,min_coords,bag_date,bag_type,bag_timestamp,file_postfix):
    #WARNING: Work in progress...
    directory = '../export_data/' + bag_date + '/YOLO_dataset/' + bag_timestamp + bag_type

def main():
    output_Video      = True
    output_Video_type = "Rectangle" #Choose "Rectangle" or "Wireframe"
    output_Rectangle  = True
    output_Wireframe  = True
    output_Coco       = True

    file_postfix = 0

    #Update these for each separate recording
    bag_date = '20190206'
    bag_timestamp = '2019-02-06-22-42-24'
    # bag_date = '20190209'
    # bag_timestamp = '2019-02-09-21-43-51'

    bag_directory = '../record_data/' + bag_date
    bag_depth = read_bag(bag_date,bag_timestamp,bag_directory,'depth')
    bag_rgb = read_bag(bag_date,bag_timestamp,bag_directory,'rgb')
    bag_pose_RealSense = read_bag(bag_date,bag_timestamp,bag_directory,'pose_RealSense')
    bag_pose_Quad = read_bag(bag_date,bag_timestamp,bag_directory,'pose_Quad')
    # bag_pose_Quad = read_bag(bag_date,bag_timestamp,bag_directory,'pose_Calibration')

    save_image_Rectangle_directory = '../export_data/' + bag_date + '/drawn_Rectangle/' + bag_timestamp
    save_image_Wireframe_directory = '../export_data/' + bag_date + '/drawn_Wireframe/' + bag_timestamp
    save_image_Coco_directory = '../export_data/' + bag_date + '/Coco/' + bag_timestamp
    videoName = '../export_data/' + bag_date + '/' + bag_timestamp + '_video.mp4'



    if output_Rectangle == True:
        cv_depth_Rectangle = Image_Processing.image_processing()
        cv_rgb_Rectangle   = Image_Processing.image_processing()
    if output_Wireframe == True:
        cv_depth_Wireframe = Image_Processing.image_processing()
        cv_rgb_Wireframe   = Image_Processing.image_processing()
    if output_Coco == True:
        cv_depth_Coco = Image_Processing.image_processing()
        cv_rgb_Coco   = Image_Processing.image_processing()
        depth_annotation_Coco = Save_to_Coco_Format.CocoFormat(save_image_Coco_directory+"_depth")
        rgb_annotation_Coco   = Save_to_Coco_Format.CocoFormat(save_image_Coco_directory+"_rgb")
    if output_Video == True:
        videoName = '../export_data/' + bag_date + '/' + bag_timestamp + '_video.mp4'
        fourcc = cv2.VideoWriter_fourcc(*'X264')
        video  = cv2.VideoWriter(videoName,fourcc,30.0,(640,480))

    # print cv_depth_Rectangle == cv_depth_Coco
    for topic_depth, msg_depth, t_depth in bag_depth.read_messages():
        t_start = t_depth-rospy.Duration(0.1)
        t_end   = t_depth+rospy.Duration(0.1)
        ## Get messages with timestamps closest to the depth's message
        msg_rgb, t_rgb                       = closest_msg(t_depth, t_start, t_end, bag_rgb)
        msg_pose_RealSense, t_pose_RealSense = closest_msg(t_depth, t_start, t_end, bag_pose_RealSense)
        msg_pose_Quad, t_pose_Quad           = closest_msg(t_depth, t_start, t_end, bag_pose_Quad)

        ## Find projected pixel coordinates
        vertex_coords, max_coords, min_coords = vertex_coordinates(msg_pose_Quad,msg_pose_RealSense)

        ## Draw rectangle on images in opencv
        if output_Rectangle == True:
            cv_depth_Rectangle.convert_to_cv(msg_depth)
            cv_rgb_Rectangle.convert_to_cv(msg_rgb)
            cv_depth_Rectangle.draw_rectangle(max_coords,min_coords)
            cv_rgb_Rectangle.draw_rectangle(max_coords,min_coords)
        if output_Wireframe == True:
            cv_depth_Wireframe.convert_to_cv(msg_depth)
            cv_rgb_Wireframe.convert_to_cv(msg_rgb)
            cv_depth_Wireframe.draw_wireframe(vertex_coords)
            cv_rgb_Wireframe.draw_wireframe(vertex_coords)
            # cv_rgb_Wireframe.draw_rectangle(image,max_coords,min_coords)
        if output_Coco == True:
            cv_depth_Coco.convert_to_cv(msg_depth)
            cv_rgb_Coco.convert_to_cv(msg_rgb)

        ## Save images
        if output_Rectangle == True:
            save_image(cv_depth_Rectangle,save_image_Rectangle_directory,'depth',file_postfix)
            save_image(cv_rgb_Rectangle  ,save_image_Rectangle_directory,'rgb'  ,file_postfix)
        if output_Wireframe == True:
            save_image(cv_depth_Wireframe,save_image_Wireframe_directory,'depth',file_postfix)
            save_image(cv_rgb_Wireframe  ,save_image_Wireframe_directory,'rgb'  ,file_postfix)
        if output_Video == True:
            if output_Video_type == "Wireframe":
                video.write(cv_rgb_Wireframe.image_cv)
            else: #default choice is Rectangle
                video.write(cv_rgb_Rectangle.image_cv)
        if output_Coco == True:
            depth_filename = save_image(cv_depth_Coco,save_image_Coco_directory,'depth',file_postfix)
            rgb_filename   = save_image(cv_rgb_Coco  ,save_image_Coco_directory,'rgb'  ,file_postfix)
            depth_annotation_Coco.image_annotation(file_postfix, depth_filename, max_coords, min_coords)
            rgb_annotation_Coco.image_annotation(file_postfix, rgb_filename, max_coords, min_coords)
        # save_image_for_YOLO(cv_depth,max_coords,min_coords,bag_date,bag_timestamp,'depth',file_postfix)
        # save_image_for_YOLO(cv_rgb,max_coords,min_coords,bag_date,bag_timestamp,'rgb',file_postfix)
        file_postfix += 1

    if output_Coco == True:
        depth_annotation_Coco.output_file()
        rgb_annotation_Coco.output_file()
    if output_Video == True:
        cv2.destroyAllWindows()
        video.release()

if __name__ == '__main__':
    main()
