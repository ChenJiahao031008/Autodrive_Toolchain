#!usr/bin/python
# -*- coding: utf-8 -*-

import time, sys, os
from ros import rosbag
import roslib
import rospy
import numpy
import cv2
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL
import natsort

# ---------------------------基本参数设置区域-------------------------------- #
## 如果更改模式，请修改此处，支持rgb模式和rgbd模式
data_mode = "rgbd"
## 如果更改深度图像话题名称，请修改此处
ros_depth_topic = 'camera/aligned_depth_to_color/image_raw'
## 如果更改彩色图像话题名称，请修改此处
ros_rgb_topic = 'camera/color/image_raw'
## 如果更改深度图像格式，请修改此处
depth_class = '16UC1'
## 如果更改时间戳赋值方式，请修改此处
## 0: 依靠图像时间戳进行赋值; 1: 依靠图像读写顺序进行赋值
use_timestamp = 0
## 如果更改读取时间的间隔，请修改此处(仅对 use_timestamp = 1 时生效)
frequency = 15
# ------------------------------------------------------------------------- #

def GetFilesFromDir(dir):
    print( "Searching directory %s" % dir )
    rgb_files = []
    depth_files = []
    if os.path.exists(dir):
        ## TODO: 如果更换目录，请在此处修改/rgb 或者 /depth
        for path, names, files in os.walk(dir + '/rgb'):
            for f in natsort.natsorted(files):
                rgb_files.append( os.path.join( path, f ) )
        for path, names, files in os.walk(dir + '/depth'):
            for f in natsort.natsorted(files):
                depth_files.append( os.path.join( path, f ) )
    return rgb_files, depth_files

def CreateRGBDBag(rgb_files, depth_files, bagname):
    cb = CvBridge()
    bag = rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(depth_files)):
            img = cv2.imread(depth_files[i], cv2.IMREAD_UNCHANGED)
            print("Adding %s" % depth_files[i])

            (time,extension) = os.path.splitext(os.path.basename(depth_files[i]))
            if (use_timestamp == 0):
                floattime = float(time)
            else:
                floattime = i*1.0/frequency
            image = cb.cv2_to_imgmsg(img, encoding=depth_class)
            image.header.stamp = rospy.rostime.Time.from_sec(floattime)
            image.header.frame_id = "camera/aligned_depth_to_color"
            if (data_mode == "rgbd"):
                bag.write(ros_depth_topic, image, rospy.rostime.Time.from_sec(floattime))

        for i in range(len(rgb_files)):
            img = cv2.imread(rgb_files[i], cv2.IMREAD_UNCHANGED)
            print("Adding %s" % rgb_files[i])

            (time,extension) = os.path.splitext(os.path.basename(rgb_files[i]))
            if (use_timestamp == 0):
                floattime = float(time)
            else:
                floattime = i*1.0/frequency

            image = cb.cv2_to_imgmsg(img, encoding='bgr8')
            image.header.stamp = rospy.rostime.Time.from_sec(floattime)
            image.header.frame_id = "camera/color"
            if (data_mode == "rgbd" or data_mode == "rgb"):
                bag.write(ros_rgb_topic, image, rospy.rostime.Time.from_sec(floattime))

    finally:
        bag.close()


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    rgb_files, depth_files = GetFilesFromDir(args[0])
    CreateRGBDBag(rgb_files, depth_files, args[1])

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateBag(sys.argv[1:])
    else:
        print( "Usage: convert2bag.py rootdir bagfilename")
        print( "Example: python convert2bag.py ./ new_bag_name.bag")
