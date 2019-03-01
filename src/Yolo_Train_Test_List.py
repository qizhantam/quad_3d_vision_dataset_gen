import os, glob

# Data to be processed
bag_date = '20190217'
bag_timestamp = '2019-02-17-01-07-30'
rgb_Yolo_directory = '../export_data/' + bag_date + '/Yolo/' + bag_timestamp + "_rgb"
write_path = os.path.abspath(rgb_Yolo_directory)
print write_path

# Percentage of images to be used for the test set
percentage_test = 10;

# Create and/or truncate train.txt and test.txt
file_train = open('./Yolo/' + 'train.txt', 'w')
file_test  = open('./Yolo/' + 'test.txt', 'w')

# Populate train.txt and test.txt
counter = 1
index_test = round(100 / percentage_test)
for pathAndFilename in glob.iglob(os.path.join(rgb_Yolo_directory, "*.png")):
    title, ext = os.path.splitext(os.path.basename(pathAndFilename))

    if counter == index_test:
        counter = 1
        file_test.write(write_path + "/" + title + '.png' + "\n")
    else:
        file_train.write(write_path + "/" + title + '.png' + "\n")
        counter = counter + 1

file_train.close()
file_test.close()
