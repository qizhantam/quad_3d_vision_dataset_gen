import os, glob
from ROSImageToCV2 import load_bag_date_timestamp
import pdb

# Data to be processed
bag_date, bag_timestamp = load_bag_date_timestamp()
rgb_Yolo_directory = '../export_data/' + bag_date + '/Yolo/' + bag_timestamp + "_rgb"
# rgb_Yolo_directory = '../export_data/' + bag_date + '/Unedited/' + bag_timestamp + "_rgb"
image_path = os.path.abspath(rgb_Yolo_directory)
print "Image path to be written to list: ", image_path

# Percentage of images to be used for the test set
percentage_test = 1.;

# Create and/or truncate train.txt and test.txt
file_train = open('./Yolo/' + 'train.txt', 'w')
file_test  = open('./Yolo/' + 'test.txt', 'w')

# Populate train.txt and test.txt
counter = 1
index_test = int(percentage_test*100.)
for pathAndFilename in glob.iglob(os.path.join(rgb_Yolo_directory, "*.png")):
    title, ext = os.path.splitext(os.path.basename(pathAndFilename))
    # if counter is not index_test:
    #     counter = counter + 1
    #     file_train.write(image_path + "/" + title + '.png' + "\n")
    # else:
    #     counter = 1
    #     file_test.write(image_path + "/" + title + '.png' + "\n")
    file_train.write(image_path + "/" + title + '.png' + "\n")
    file_test.write(image_path + "/" + title + '.png' + "\n")



file_train.close()
file_test.close()
