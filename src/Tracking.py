#!/usr/bin/env python
import cv2, rospy
import numpy as np
import Image_Processing
from File_Operations import read_bag, save_image, closest_msg, low_pass_filter, label_text
from Coordinate_Transformations import vertex_coordinates, ground_truth_center, vertex_coordinates_EKF
from EKF import EKF_v1, EKF_v1_noMeasureUpdate, return_u_t
from ROSImageToCV2 import load_bag_date_timestamp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, writers
from natsort import natsorted

# def load_bag_date_timestamp():
#     bag_date = '20190309'
#
#     #NOTE: Change these for different rosbag files
#     # bag_timestamp = '2019-03-09-12-19-21'
#     bag_timestamp = '2019-03-09-12-14-29'
#     return bag_date, bag_timestamp

def main():
    Sig_t = np.array([\
                [1.,0.,0.],\
                [0.,1.,0.],\
                [0.,0.,1.]])
    Sig_t = Sig_t*0.

    bag_date, bag_timestamp = load_bag_date_timestamp()

    bag_directory = '../record_data/' + bag_date
    bag_rgb = read_bag(bag_date,bag_timestamp,bag_directory,'rgb')
    bag_pose_RealSense = read_bag(bag_date,bag_timestamp,bag_directory,'pose_RealSense')
    bag_pose_Quad = read_bag(bag_date,bag_timestamp,bag_directory,'pose_Quad')

    save_image_Wireframe_directory = '../export_data/' + bag_date + '/EKF/' + bag_timestamp
    bbox_file_directory = './Yolo/results/results_' + bag_timestamp + '.txt'
    videoName = '../export_data/' + bag_date + '/' + bag_timestamp + '_EKF_video.mp4'

    #NOTE: Uncomment and change path for recording plot animation.
    # videoName2 = '../export_data/' + bag_date + '/' + bag_timestamp + '_EKF_plot_video.mp4'

    bbox_file = open(bbox_file_directory,'r')
    bbox_lines = bbox_file.readlines()
    bbox_lines = natsorted(bbox_lines)
    bbox_img_list = []
    bbox_confidence_list = []
    bbox_xmax_list = []
    bbox_ymax_list = []
    bbox_xmin_list = []
    bbox_ymin_list = []
    bbox_index = int(0)
    for bbox_line in bbox_lines:
        bbox_img, bbox_confidence, bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax = bbox_line.split(" ")
        _ , img_num = bbox_img.split("_")
        bbox_img_list.append(int(img_num))
        bbox_confidence_list.append(float(bbox_confidence))
        bbox_xmax_list.append(float(bbox_xmax))
        bbox_ymax_list.append(float(bbox_ymax))
        bbox_xmin_list.append(float(bbox_xmin))
        bbox_ymin_list.append(float(bbox_ymin))

    cv_rgb_Wireframe   = Image_Processing.image_processing()
    fourcc = cv2.VideoWriter_fourcc(*'X264')
    #NOTE: Decrease/Increase frame rate to change playback speed. 15 is the normal speed.
    video  = cv2.VideoWriter(videoName,fourcc,15.0,(640,480))

    #NOTE: Uncomment to record plot animation (Also uncomment lines at the bottom)
    # video2  = cv2.VideoWriter(videoName2,fourcc,15.0,(640,480))

    t_ros = [];

    for topic_rgb, msg_rgb, t_rgb in bag_rgb.read_messages():
        t_ros.append(t_rgb)

    dt = 0.01
    t_mat = np.arange(0.,(t_ros[-1]-t_ros[0]).to_sec()+dt,dt)

    t_ros_index = int(0)
    dist_truth = []
    dist_EKF = []
    t_plot = []
    for index, t_now in enumerate(t_mat):
        rgb_lag_time = 0.2 #how far behind is rgb's actual timestamp vs. pose timestamp
        t_ros_now = t_ros[0]+rospy.Duration(t_now)
        t_start = t_ros_now-rospy.Duration(0.3)
        t_end   = t_ros_now+rospy.Duration(0.1)
        t_start_rgb = t_ros_now-rospy.Duration(0.1)
        t_end_rgb = t_ros_now+rospy.Duration(0.1)

        msg_pose_RealSense, t_pose_RealSense = closest_msg(t_ros_now, t_start, t_end, bag_pose_RealSense, rgb_lag_time)
        msg_pose_Quad, t_pose_Quad           = closest_msg(t_ros_now, t_start, t_end, bag_pose_Quad     , rgb_lag_time)

        quad_position_truth, quad_orientation_truth = ground_truth_center(msg_pose_Quad,msg_pose_RealSense)
        quad_pose_truth = quad_position_truth

        if index == 0: #Initialization of quad pose and orientation
            quad_pose_t = quad_pose_truth #begin EKF with ground truth
            quad_pose_t_old = quad_pose_t
            quad_orientation = quad_orientation_truth #fix orientation with the beginning orientation (not updated with EKF)

        u_t = return_u_t(quad_pose_t,quad_pose_t_old,dt)

        if t_ros_now >= t_ros[t_ros_index]:
            print "index: ", t_ros_index
            msg_rgb, t_rgb                   = closest_msg(t_ros_now, t_start, t_end, bag_rgb, 0.)

            #NOTE: Ground Truth of 3D bbox vertices
            # vertex_coords, max_coords, min_coords = vertex_coordinates(msg_pose_Quad,msg_pose_RealSense)

            cv_rgb_Wireframe.convert_to_cv(msg_rgb,"rgb")

            if bbox_index == np.size(bbox_img_list):
                break

            if t_ros_index == bbox_img_list[bbox_index]: #if there is z_t, i.e. bbox dimensions available
                px = (bbox_xmax_list[bbox_index]+bbox_xmin_list[bbox_index])/2.
                py = (bbox_ymax_list[bbox_index]+bbox_ymin_list[bbox_index])/2.
                dx = (bbox_xmax_list[bbox_index]-bbox_xmin_list[bbox_index])
                dy = (bbox_ymax_list[bbox_index]-bbox_ymin_list[bbox_index])
                z_t = np.array([[px,py,dx,dy]]).T
                quad_pose_t[:3], Sig_t = EKF_v1(quad_pose_t[:3],Sig_t,u_t,z_t,msg_pose_RealSense)
                bbox_max = [bbox_xmax_list[bbox_index],bbox_ymax_list[bbox_index]]
                bbox_min = [bbox_xmin_list[bbox_index],bbox_ymin_list[bbox_index]]
                cv_rgb_Wireframe.overlay(bbox_max,bbox_min)
                bbox_index += 1
            else:
                quad_pose_t[:3], Sig_t = EKF_v1_noMeasureUpdate(quad_pose_t[:3],Sig_t,u_t)

            #NOTE: Estimated 3D bbox vertices from Yolo 2D bbox
            vertex_coords = vertex_coordinates_EKF(quad_pose_t,quad_orientation,msg_pose_RealSense)

            cv_rgb_Wireframe.draw_wireframe_EKF(vertex_coords)

            #NOTE: Uncomment to show each individual frame while being looped for debugging
            # cv2.imshow('img',cv_rgb_Wireframe.image_cv)
            # cv2.waitKey(0)

            video.write(cv_rgb_Wireframe.image_cv)
            t_ros_index += 1

            dist_truth.append(np.sqrt(quad_pose_truth[0][0]**2+quad_pose_truth[1][0]**2+quad_pose_truth[2][0]**2))
            dist_EKF.append(np.sqrt(quad_pose_t[0][0]**2+quad_pose_t[1][0]**2+quad_pose_t[2][0]**2))
            t_plot.append(t_now)

        else:
            quad_pose_t[:3], Sig_t = EKF_v1_noMeasureUpdate(quad_pose_t[:3],Sig_t,u_t)

        quad_pose_t_old = quad_pose_t
        cv2.destroyAllWindows()
    video.release()

    #NOTE: Comment this paragraph before recording plot animation
    linewidth = 3
    fontsize = 20
    fig, ax = plt.subplots()
    ax.set_xlim(0,t_plot[-1])
    plt.tick_params(labelsize=20)
    ax.set_ylim(0,np.max([np.max(dist_EKF),np.max(dist_truth)]))
    ln, = plt.plot(t_plot,dist_EKF,label='EKF',color = (1,0,0),linewidth=linewidth)
    ln2, = plt.plot(t_plot,dist_truth,label='Ground Truth',color = (0,0,1),linewidth=linewidth)
    plt.xlabel('Time (s)',fontsize=fontsize)
    plt.ylabel('Distance (m)',fontsize=fontsize)
    plt.title('Relative Distance to Camera',fontsize=fontsize)
    plt.legend(fontsize=fontsize-5,loc='upper left')
    plt.show()

    #NOTE: Uncomment to record plot animation (Also uncomment lines at the top)
    # linewidth = 3
    # fontsize = 20
    # fig, ax = plt.subplots()
    # ax.set_xlim(0,t_plot[-1])
    # plt.tick_params(labelsize=20)
    # ax.set_ylim(0,np.max([np.max(dist_EKF),np.max(dist_truth)]))
    # ln, = plt.plot(t_plot,dist_EKF,label='EKF',color = (1,0,0),linewidth=linewidth)
    # ln2, = plt.plot(t_plot,dist_truth,label='Ground Truth',color = (0,0,1),linewidth=linewidth)
    # plt.xlabel('Time (s)',fontsize=fontsize)
    # plt.ylabel('Distance (m)',fontsize=fontsize)
    # plt.title('Relative Distance to Camera',fontsize=fontsize)
    # plt.legend(fontsize=fontsize-5,loc='upper left')
    # def update(frame):
    #     ln.set_data(t_plot[:frame+1],dist_EKF[:frame+1])
    #     ln2.set_data(t_plot[:frame+1],dist_truth[:frame+1])
    #     return tuple([ln,ln2])
    # ani = FuncAnimation(fig,update,frames=range(np.size(t_plot)),interval = 66)
    # ani.save(videoName2,fps=60,bitrate=1800)

if __name__ == '__main__':
    main()
