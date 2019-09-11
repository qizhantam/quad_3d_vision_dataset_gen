#!/usr/bin/env python
import cv2, rospy
import Image_Processing
import Save_to_Coco_Format
from File_Operations import read_bag, save_image, closest_msg, low_pass_filter, label_text
from Coordinate_Transformations import vertex_coordinates
import pdb
import os

def load_bag_date_timestamp():
    # bag_date = '20190309'
    bag_date = '20190910'

    # bag_timestamp = '2019-03-09-12-19-21'  # small sequence (24 images)
    # bag_timestamp = '2019-03-09-12-14-29'  # training data
    bag_timestamp = '2019-09-10-12-50-30'  # gentle, small bounding box
    # bag_timestamp = '2019-09-10-16-58-50'  # sharp turns
    # bag_timestamp = '2019-09-10-17-01-41'  # yawing
    return bag_date, bag_timestamp

def main():
    output_Video      = False
    output_Video_type = "Unedited" #Choose "Rectangle" or "Wireframe" or "Unedited"
    output_Rectangle  = True
    output_Wireframe  = False
    output_NoDrawing  = True
    output_Time_and_Pose = True
    time_0 = None
    output_Coco       = False
    filter            = False

    file_postfix = 0

    #Update these for each separate recording
    bag_date, bag_timestamp = load_bag_date_timestamp()

    bag_directory = '../record_data/' + bag_date
    # bag_depth = read_bag(bag_date,bag_timestamp,bag_directory,'depth')
    bag_rgb = read_bag(bag_date,bag_timestamp,bag_directory,'rgb')
    bag_pose_RealSense = read_bag(bag_date,bag_timestamp,bag_directory,'pose_RealSense')
    bag_pose_Quad = read_bag(bag_date,bag_timestamp,bag_directory,'pose_Quad')
    # bag_pose_Quad = read_bag(bag_date,bag_timestamp,bag_directory,'pose_Calibration')

    save_gt_dir = './Yolo/results/results_' + bag_timestamp + '/pose_gtboxes_and_time'

    save_image_Rectangle_directory = '../export_data/' + bag_date + '/drawn_Rectangle/' + bag_timestamp
    save_image_Wireframe_directory = '../export_data/' + bag_date + '/drawn_Wireframe/' + bag_timestamp
    save_image_NoDrawing_directory = '../export_data/' + bag_date + '/Unedited/' + bag_timestamp
    save_image_Coco_directory = '../export_data/' + bag_date + '/Coco/' + bag_timestamp
    save_label_text_directory = '../export_data/' + bag_date + '/Labels/' + bag_timestamp
    videoName = '../export_data/' + bag_date + '/' + bag_timestamp + '_video.mp4'
    # videoName_depth = '../export_data/' + bag_date + '/' + bag_timestamp + '_depth_video.mp4'

    fc = 100 #Hz, Cutoff frequency for low-pass filter
    alpha = 1./(1.+30./(fc*3.142))
    if filter == True:
        bag_pose_Quad = low_pass_filter(alpha,bag_date,bag_timestamp,bag_directory,'pose_Quad')

    if output_Rectangle == True:
        # cv_depth_Rectangle = Image_Processing.image_processing()
        cv_rgb_Rectangle   = Image_Processing.image_processing()
    if output_Wireframe == True:
        # cv_depth_Wireframe = Image_Processing.image_processing()
        cv_rgb_Wireframe   = Image_Processing.image_processing()
    if output_NoDrawing == True:
        # cv_depth_NoDrawing = Image_Processing.image_processing()
        cv_rgb_NoDrawing   = Image_Processing.image_processing()
    if output_Coco == True:
        # cv_depth_Coco = Image_Processing.image_processing()
        cv_rgb_Coco   = Image_Processing.image_processing()
        depth_annotation_Coco = Save_to_Coco_Format.CocoFormat(save_image_Coco_directory+"_depth")
        rgb_annotation_Coco   = Save_to_Coco_Format.CocoFormat(save_image_Coco_directory+"_rgb")
    if output_Video == True:
        videoName = '../export_data/' + bag_date + '/' + bag_timestamp + '_video.mp4'
        fourcc = cv2.VideoWriter_fourcc(*'X264')
        video  = cv2.VideoWriter(videoName,fourcc,60.0,(640,480))
        # video_depth = cv2.VideoWriter(videoName_depth,fourcc,60.0,(640,480))

    # print cv_depth_Rectangle == cv_depth_Coco
    ind = 0
    for topic_depth, msg_depth, t_depth in bag_rgb.read_messages():
        # if ind > 300:
        #     break
        # ind += 1
        t_start = t_depth-rospy.Duration(0.3)
        t_end   = t_depth+rospy.Duration(0.1)
        t_start_rgb = t_depth-rospy.Duration(0.1)
        t_end_rgb = t_depth+rospy.Duration(0.1)
        ## Get messages with timestamps closest to the depth's message
        rgb_lag_time = 0 #0.2 #compensate how far behind is rgb's actual timestamp vs. pose timestamp
        # rgb_lag_time = 0.2 #compensate how far behind is rgb's actual timestamp vs. pose timestamp
        msg_rgb, t_rgb                       = closest_msg(t_depth, t_start, t_end, bag_rgb, 0.)
        msg_pose_RealSense, t_pose_RealSense = closest_msg(t_depth, t_start, t_end, bag_pose_RealSense, rgb_lag_time)
        msg_pose_Quad, t_pose_Quad           = closest_msg(t_depth, t_start, t_end, bag_pose_Quad, rgb_lag_time)

        if file_postfix == 0:
            time_0 = min([t_pose_Quad.secs  + t_pose_Quad.nsecs*10**-9, t_start_rgb.secs  + t_start_rgb.nsecs*10**-9])
            # pdb.set_trace()
        ## Find projected pixel coordinates
        # pdb.set_trace()
        vertex_coords, max_coords, min_coords = vertex_coordinates(msg_pose_Quad,msg_pose_RealSense)

        ## Draw rectangle on images in opencv
        if output_Rectangle == True:
            # cv_depth_Rectangle.convert_to_cv(msg_depth,"depth")
            cv_rgb_Rectangle.convert_to_cv(msg_rgb,"rgb")
            # cv_depth_Rectangle.draw_rectangle(max_coords,min_coords)
            cv_rgb_Rectangle.draw_rectangle(max_coords,min_coords)
        if output_Wireframe == True:
            # cv_depth_Wireframe.convert_to_cv(msg_depth,"depth")
            cv_rgb_Wireframe.convert_to_cv(msg_rgb,"rgb")
            # cv_depth_Wireframe.draw_wireframe(vertex_coords)
            cv_rgb_Wireframe.draw_wireframe(vertex_coords)
            # cv_rgb_Wireframe.draw_rectangle(image,max_coords,min_coords)
        if output_NoDrawing == True:
            # cv_depth_NoDrawing.convert_to_cv(msg_depth,"depth")
            cv_rgb_NoDrawing.convert_to_cv(msg_rgb,"rgb")
        if output_Coco == True:
            # cv_depth_Coco.convert_to_cv(msg_depth,"depth")
            cv_rgb_Coco.convert_to_cv(msg_rgb,"rgb")

        ## Save images
        if output_Rectangle == True:
            # save_image(cv_depth_Rectangle,save_image_Rectangle_directory,'depth',file_postfix)
            save_image(cv_rgb_Rectangle  ,save_image_Rectangle_directory,'rgb'  ,file_postfix)
        if output_Wireframe == True:
            # save_image(cv_depth_Wireframe,save_image_Wireframe_directory,'depth',file_postfix)
            save_image(cv_rgb_Wireframe  ,save_image_Wireframe_directory,'rgb'  ,file_postfix)
        if output_NoDrawing == True:
            # depth_filename = save_image(cv_depth_NoDrawing,save_image_NoDrawing_directory,'depth',file_postfix)
            rgb_filename = save_image(cv_rgb_NoDrawing    ,save_image_NoDrawing_directory,'rgb'  ,file_postfix)
            # label_text(save_label_text_directory, file_postfix, max_coords, min_coords, 'depth')
            label_text(save_label_text_directory, file_postfix, max_coords, min_coords, 'rgb')
        
        if output_Time_and_Pose == True:
            file_type = "pose_gtboxes_and_time"
            if not os.path.exists(save_gt_dir):
                os.makedirs(save_gt_dir)
            filename = save_gt_dir + '/' + file_type + ('_%i.txt' % (file_postfix))
            # pdb.set_trace()
            file = open(filename,'w')
            write_str = "{:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f} {:f}".format(
                t_rgb.secs + t_rgb.nsecs*10**-9 - time_0,
                t_pose_Quad.secs + t_pose_Quad.nsecs*10**-9 - time_0,
                msg_pose_Quad.pose.position.x, 
                msg_pose_Quad.pose.position.y, 
                msg_pose_Quad.pose.position.z,
                msg_pose_Quad.pose.orientation.w, 
                msg_pose_Quad.pose.orientation.x, 
                msg_pose_Quad.pose.orientation.y, 
                msg_pose_Quad.pose.orientation.z,
                max_coords[0], max_coords[1],
                min_coords[0], min_coords[1])
            file.write(write_str)
            file.close()


        if output_Video == True:
            if output_Video_type == "Wireframe":
                video.write(cv_rgb_Wireframe.image_cv)
            elif output_Video_type == "Rectangle": #default choice is Rectangle
                video.write(cv_rgb_Rectangle.image_cv)
            elif output_Video_type == "Rectangle":
                video_depth.write(cv_depth_NoDrawing.image_cv)
                video.write(cv_rgb_NoDrawing.image_cv)
        if output_Coco == True:
            # depth_filename = save_image(cv_depth_Coco,save_image_Coco_directory,'depth',file_postfix)
            rgb_filename   = save_image(cv_rgb_Coco  ,save_image_Coco_directory,'rgb'  ,file_postfix)
            depth_annotation_Coco.image_annotation(file_postfix, depth_filename, max_coords, min_coords)
            rgb_annotation_Coco.image_annotation(file_postfix, rgb_filename, max_coords, min_coords)
        file_postfix += 1

    if output_Coco == True:
        depth_annotation_Coco.output_file()
        rgb_annotation_Coco.output_file()
    if output_Video == True:
        cv2.destroyAllWindows()
        video.release()

if __name__ == '__main__':
    main()