import os, rosbag, rospy, cv2

def read_bag(bag_date,bag_timestamp,bag_directory,bag_type):
    return rosbag.Bag(bag_directory + '/' + bag_type + '_' + bag_timestamp + '.bag')

def save_image(cv_object,save_directory,bag_type,file_postfix):
    directory = save_directory + "_" + bag_type
    if not os.path.exists(directory):
      os.makedirs(directory)
    filename = directory + '/' + bag_type + ('_%i.png' % (file_postfix))
    cv2.imwrite(filename,cv_object.image_cv)
    return bag_type + ('_%i.png' % (file_postfix))

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
