import numpy as np
import os
import matplotlib as mlp
import matplotlib.pyplot as plt

##############################################
# 代码功能：读取的两个文件，只保留前两个数据进行绘图 #
##############################################

## 需要读取的两个文件，如果更改请修改此处
file_name_1 = "./MC.txt"
file_name_2 = "./NMC.txt"

## ________________读取文件________________ ##

vel_x_1 = []
vel_y_1 = []
vel_x_2 = []
vel_y_2 = []

file1 = open(file_name_1, 'r')
for line in file1.readlines():
    line = line.strip('\n')        # 除去换行
    line = line.split(' ')         # 文件以“ ”分隔
    if "" in line:                 # 解决每行结尾有空格的问题
        line.remove("")
    vel_x_1.append(float(line[0]))
    vel_y_1.append(float(line[1]))

file2 = open(file_name_2, 'r')
for line in file2.readlines():
    line = line.strip('\n')        # 除去换行
    line = line.split(' ')         # 文件以“ ”分隔
    if "" in line:                 # 解决每行结尾有空格的问题
        line.remove("")
    vel_x_2.append(float(line[0]))
    vel_y_2.append(float(line[1]))

## ________________绘制图片________________ ##
num = min(len(vel_y_1), len(vel_y_2))
idx = [i for i in range(num)]

vel_x_1 = vel_x_1[0:num]
vel_x_2 = vel_x_2[0:num]
vel_y_1 = vel_y_1[0:num]
vel_y_2 = vel_y_2[0:num]

plt.figure(1)
plt.plot(idx,vel_y_1, color="blue",linewidth=1.0,linestyle="-",label="vel_y_1",alpha=0.9)    # 颜色 线宽 类型 标签 透明度
plt.plot(idx,vel_y_2, color="red",linewidth=1.0,linestyle="-",label="vel_y_2",alpha=0.9)    # 颜色 线宽 类型 标签 透明度
# plt.plot(idx,vel_z_1, color="blue", linewidth=1.0,linestyle="-",label="vel_z",alpha=0.9)    # 颜色 线宽 类型 标签 透明度
# plt.plot(idx,vel_z_2, color="red", linewidth=1.0,linestyle="-",label="vel_z",alpha=0.9)    # 颜色 线宽 类型 标签 透明度
# plt.legend(loc="upper left") # 图例
plt.grid() # 网格线
plt.show()
