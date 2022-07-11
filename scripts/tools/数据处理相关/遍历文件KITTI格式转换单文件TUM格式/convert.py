## 根据是否去除ros的python版本
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import random
import glob
import os

import cv2
import math
import numpy as np
from pyquaternion import Quaternion

#############################################################################
# 使用方法:
## python convert.py
# ———————————————————————————————————————————————————————————————————————————
# 待调整参数：
## files_path为KITTI数据集格式的位姿文件夹
files_path = glob.glob(os.path.join("./pose","*.txt"))
## output_file为TUM格式的位姿文件
output_file = 'ground_truth.txt'
##############################################################################

files_path.sort()
results = []
for id_number,file_name in enumerate(files_path):
    file = open(file_name, 'r')
    rotate_matrix = []
    translation = []
    for index, line in enumerate(file.readlines()):
        line = line.strip('\n')        # 除去换行
        line = line.split('\t')        # 文件以“ ”分隔
        if "" in line:                 # 解决每行结尾有空格的问题
            line.remove("")
        if (index!=3):
            rotate_matrix.append(np.array([float(line[0]), float(line[1]), float(line[2])]))
            translation.append(float(line[3]))
    R = np.array(rotate_matrix)
    t = np.array(translation)
    q = Quaternion(matrix=R,atol=1e-03)
    ftime = id_number*100.0/1500.0
    print(ftime, q.w, q.x, q.y, q.z, t[0], t[1], t[2])
    results.append(['%.6f' % ftime, t[0], t[1], t[2], q.x, q.y, q.z, q.w])
    print("------------------")

with open(output_file,'w') as out:
    for res in results:
        line = str(res[0]) + " " + str(res[1]) + " " + str(res[2]) + " " + str(
                res[3]) + " " + str(res[4]) + " " + str(res[5]) + " " + str(res[6]) + " " + str(res[7]) + "\n"
        out.write(line)
