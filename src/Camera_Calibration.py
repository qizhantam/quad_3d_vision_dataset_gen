import numpy as np
import cv2 as cv
import glob
from Calibration_test import read_bag, closest_msg
from cv_bridge import CvBridge
import rospy, tf
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*9,3), np.float32)

x_delta = -17.35/1000. #m
y_delta = x_delta

x0 = -0.942589958080699+x_delta/2. #m
y0 = 0.47854556109716095+y_delta/2. #m
z0 = 0.021640986672089045 #m

x1 = x0+8.*x_delta#m
y1 = y0+8.*y_delta #m

objp[:,2] = z0

grid1 = objp
grid2 = objp
grid3 = objp
grid4 = objp
grid5 = objp
grid6 = objp
grid7 = objp
grid8 = objp
grid9 = objp
grid10 = objp
grid11 = objp
grid12 = objp
grid13 = objp
grid14 = objp
grid15 = objp
grid16 = objp

grid1[:,:2]  = np.mgrid[x1:x0-x_delta/2.:-x_delta,y0:y1+y_delta/2.:y_delta].T.reshape(-1,2) #253
grid2[:,:2]  = np.mgrid[y1:y0-y_delta/2.:-y_delta,x1:x0-x_delta/2.:-x_delta].T.reshape(-1,2) #368
grid3[:,:2]  = np.mgrid[y1:y0-y_delta/2.:-y_delta,x1:x0-x_delta/2.:-x_delta].T.reshape(-1,2) #758
grid4[:,:2]  = np.mgrid[y1:y0-y_delta/2.:-y_delta,x1:x0-x_delta/2.:-x_delta].T.reshape(-1,2) #861
grid5[:,:2]  = np.mgrid[x0:x1+x_delta/2.:x_delta,y1:y0-y_delta/2.:-y_delta].T.reshape(-1,2) #972
grid6[:,:2]  = np.mgrid[x0:x1+x_delta/2.:x_delta,y1:y0-y_delta/2.:-y_delta].T.reshape(-1,2) #1043
grid7[:,:2]  = np.mgrid[y1:y0-y_delta/2.:-y_delta,x1:x0-x_delta/2.:-dx_delta].T.reshape(-1,2) #1088

grid8[:,:2]  = np.mgrid[y0:y1+y_delta/2.:y_delta,x0:x1+x_delta/2.:x_delta].T.reshape(-1,2) #1169
grid9[:,:2]  = np.mgrid[y0:y1+y_delta/2.:y_delta,x0:x1+x_delta/2.:x_delta].T.reshape(-1,2) #1280
grid10[:,:2] = np.mgrid[y0:y1+y_delta/2.:y_delta,x0:x1+x_delta/2.:x_delta].T.reshape(-1,2) #1369
grid11[:,:2] = np.mgrid[x1:x0-x_delta/2.:-x_delta,y0:y1+y_delta/2.:y_delta].T.reshape(-1,2) #1464
grid12[:,:2] = np.mgrid[y1:y0-y_delta/2.:-y_delta,x1:x0-x_delta/2.:-x_delta].T.reshape(-1,2) #1564
grid13[:,:2] = np.mgrid[x0:x1+x_delta/2.:x_delta,y1:y0-y_delta/2.:-y_delta].T.reshape(-1,2) #1689
grid14[:,:2] = np.mgrid[x0:x1+x_delta/2.:x_delta,y1:y0-y_delta/2.:-y_delta].T.reshape(-1,2) #1861
grid15[:,:2] = np.mgrid[y1:y0-y_delta/2.:-y_delta,x1:x0-x_delta/2.:-x_delta].T.reshape(-1,2) #1914
grid16[:,:2] = np.mgrid[y1:y0-y_delta/2.:-y_delta,x1:x0-x_delta/2.:-x_delta].T.reshape(-1,2) #2011

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

objpoints_candidates = [grid1,grid2,grid3,grid4,grid5,grid6,grid7,grid8,grid9,grid10,grid11,grid12,grid13,grid14,grid15,grid16]
#Update these for each separate recording
bag_date = '20190209'
bag_timestamp = '2019-02-09-21-43-51'
choose_files = np.array([253,368,758,861,972,1043,1088,1169,1280,1369,1464,1564,1689,1861,1914,2011])
choose_files_index = 0
file_postfix = 0

bag_directory = '../record_data/' + bag_date
bag_depth = read_bag(bag_date,bag_timestamp,bag_directory,'depth')
bag_rgb = read_bag(bag_date,bag_timestamp,bag_directory,'rgb')
bag_pose_RealSense = read_bag(bag_date,bag_timestamp,bag_directory,'pose_RealSense')
bag_pose_Calibration = read_bag(bag_date,bag_timestamp,bag_directory,'pose_Calibration')

objpoints_candidates_index = 0
for topic_depth, msg_depth, t_depth in bag_depth.read_messages():

    # print file_postfix
    file_postfix += 1

    if choose_files_index>=np.size(choose_files):
        break

    if file_postfix != choose_files[choose_files_index] or choose_files_index>=np.size(choose_files):
        # print "i skipped!:", file_postfix, choose_files[choose_files_index]
        continue
    # print file_postfix
    choose_files_index += 1
    t_start = t_depth-rospy.Duration(0.1)
    t_end   = t_depth+rospy.Duration(0.1)
    # ## Get messages with timestamps closest to the depth's message
    msg_rgb, t_rgb = closest_msg(t_depth,t_start,t_end,bag_rgb)
    # msg_pose_RealSense, t_pose_RealSense = closest_msg(t_depth,t_start,t_end,bag_pose_RealSense)
    # msg_pose_Calibration, t_pose_Calibration= closest_msg(t_depth,t_start,t_end,bag_pose_Calibration)
    cv_image = CvBridge()
    image_cv = cv_image.imgmsg_to_cv2(msg_rgb)
    gray = cv.cvtColor(image_cv, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,9), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        # print file_postfix, objpoints_candidates_index
        objpoints.append(objpoints_candidates[objpoints_candidates_index])
        # print(objpoints)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(image_cv, (9,9), corners2, ret)
        cv.imshow('img', image_cv)
        print objpoints_candidates_index
        cv.waitKey(0)
    objpoints_candidates_index += 1

cv.destroyAllWindows()

K = np.array(
    [[617.2744140625, 0.0,              324.1011047363281],
     [0.0,            617.335693359375, 241.5790557861328],
     [0.0,            0.0,              1.0]]) #Intrinsic Parameters of Camera

# ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], K, None, flags=cv.CALIB_USE_INTRINSIC_GUESS)
print ret
print mtx
print dist
print np.size(rvecs)
print np.size(tvecs)

mean_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

choose_files_index = 0
objpoints_candidates_index = 0
file_postfix = 0

for topic_depth, msg_depth, t_depth in bag_depth.read_messages():
    # print file_postfix
    file_postfix += 1

    if choose_files_index>=np.size(choose_files):
        break

    if file_postfix != choose_files[choose_files_index] or choose_files_index>=np.size(choose_files):
        # print "i skipped!:", file_postfix, choose_files[choose_files_index]
        continue
    # print file_postfix
    choose_files_index += 1

    t_start = t_depth-rospy.Duration(0.1)
    t_end   = t_depth+rospy.Duration(0.1)

    ## Get messages with timestamps closest to the depth's message
    msg_pose_RealSense, t_pose_RealSense = closest_msg(t_depth,t_start,t_end,bag_pose_RealSense)

    # print rvecs[objpoints_candidates_index]
    t = tvecs[objpoints_candidates_index]
    R2 = cv.Rodrigues(rvecs[objpoints_candidates_index])
    R2 = R2[0]
    # print R2

    quaternion = (
        msg_pose_RealSense.pose.orientation.x,
        msg_pose_RealSense.pose.orientation.y,
        msg_pose_RealSense.pose.orientation.z,
        msg_pose_RealSense.pose.orientation.w)
    R = tf.transformations.quaternion_matrix(quaternion)
    R = R[:3,:3]

    C = np.array(
        [[msg_pose_RealSense.pose.position.x],
         [msg_pose_RealSense.pose.position.y],
         [msg_pose_RealSense.pose.position.z]]) #Translation Matrix

    # R_delta = np.dot(R2,np.linalg.inv(R))
    R_delta = np.dot(R2,R)
    t_w = np.dot(R.T,-C)
    t_delta = t-np.dot(R_delta,t_w)
    print "R= ", R_delta
    print "t= ", t_delta
    objpoints_candidates_index += 1
