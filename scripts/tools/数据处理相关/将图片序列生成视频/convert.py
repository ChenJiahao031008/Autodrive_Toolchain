## 根据需要选择是否屏蔽ros版本的python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import os
import cv2
import natsort

# ———————————————————————————————————————————————————————————————————————————————— #
# 默认参数配置，如果需要请在下面区域进行修改
## 要被合成的多张图片所在文件夹
file_dir = '/home/chen/Datasets/rgbd_dataset_freiburg3_long_office_household/rgb/'
## 输出的文件名称
file_output = file_dir + '../res.avi'
## 视频相关参数：图像长，宽，频率
video_width = 640
video_height = 480
video_frequency = 15
# ———————————————————————————————————————————————————————————————————————————————— #
list = []
for root ,dirs, files in os.walk(file_dir):
    for file in files:
        list.append(file)

list = natsort.natsorted(list)
# 'MJPG'意思是支持jpg格式图片
video = cv2.VideoWriter(file_output, cv2.VideoWriter_fourcc(*'MJPG'),\
    video_frequency,(video_width,video_height))

for i in range(0,len(list)):
    print(list[i])
    img = cv2.imread(file_dir + list[i])
    img = cv2.resize(img,(video_width,video_height))
    video.write(img)

video.release()
