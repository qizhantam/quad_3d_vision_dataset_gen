import os, rosbag, rospy, cv2
from shutil import copyfile

def first_order_filter(x0,x1,alpha):
    return x0 + alpha*(x1-x0)

def read_bag(bag_date,bag_timestamp,bag_directory,bag_type):
    return rosbag.Bag(bag_directory + '/' + bag_type + '_' + bag_timestamp + '.bag')

def low_pass_filter(alpha,bag_date,bag_timestamp,bag_directory,bag_type):
    outbag = rosbag.Bag(bag_directory + '/' + bag_type + '_' + bag_timestamp + '_low_pass_filter.bag','w')
    i = 0
    for topic, msg, t in rosbag.Bag(bag_directory + '/' + bag_type + '_' + bag_timestamp + '.bag').read_messages():
        if i == 0:
            ox_0 = alpha*msg.pose.orientation.x
            oy_0 = alpha*msg.pose.orientation.y
            oz_0 = alpha*msg.pose.orientation.z
            ow_0 = alpha*msg.pose.orientation.w
        else:
            ox_0 = first_order_filter(ox_0,msg.pose.orientation.x,alpha)
            oy_0 = first_order_filter(oy_0,msg.pose.orientation.y,alpha)
            oz_0 = first_order_filter(ow_0,msg.pose.orientation.z,alpha)
            ow_0 = first_order_filter(oz_0,msg.pose.orientation.w,alpha)

        msg.pose.orientation.x = ox_0
        msg.pose.orientation.y = oy_0
        msg.pose.orientation.z = oz_0
        msg.pose.orientation.w = ow_0
        outbag.write(topic,msg,t)
        i += 1
    return outbag

def save_image(cv_object,save_directory,bag_type,file_postfix):
    directory = save_directory + "_" + bag_type
    if not os.path.exists(directory):
      os.makedirs(directory)
    filename = directory + '/' + bag_type + ('_%i.png' % (file_postfix))
    cv2.imwrite(filename,cv_object.image_cv)
    return bag_type + ('_%i.png' % (file_postfix))

def copy_image(image_file,save_directory,image_type,file_postfix_png):
    directory = save_directory + "_" + image_type
    if not os.path.exists(directory):
      os.makedirs(directory)
    copyfile(image_file,directory + "/" + image_type + "_" + file_postfix_png)

def closest_msg(t_compare,t_start,t_end,bag,lag=0):
    #initialize with something ridiculous
    t_diff_old = rospy.Duration(-10000)
    t_old = rospy.Duration(-10000)

    #parse though the bag to find the message that matches most closely to input time
    for topic, msg, t in bag.read_messages(start_time=t_start,end_time=t_end):
        t_diff = t - t_compare + rospy.Duration(lag)
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

def label_text(save_directory,  file_postfix, max_coords, min_coords, file_type):
    directory = save_directory + "_" + file_type
    if not os.path.exists(directory):
      os.makedirs(directory)
    filename = directory + '/' + file_type + ('_%i.txt' % (file_postfix))
    file = open(filename,'w')
    max_str = ' '.join(map(str, max_coords))
    min_str = ' '.join(map(str, min_coords))
    file.write("0")
    file.write(" " + str(max_str) + " " + str(min_str))
    file.close()
