import numpy as np
import os
import matplotlib as mlp
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
##############################################
# 代码功能：读取的文件，保留数据进行绘图
##############################################

## 需要读取的文件，如果更改请修改此处
file_name = "../results/recorder.txt"

## ________________数据生成________________ ##
def function(x1, x2):
    z = (np.square(1 - x1) + 100 * np.square(x2 - np.square(x1)))
    return z

## ________________读取文件________________ ##

x1 = []
x2 = []
f  = []

file = open(file_name, 'r')
for line in file.readlines():
    line = line.strip('\n')        # 除去换行
    line = line.split(' ')         # 文件以“ ”分隔
    if "" in line:                 # 解决每行结尾有空格的问题
        line.remove("")
    x1.append(float(line[1]))
    x2.append(float(line[2]))
    f.append(float(line[3]))

## ________________绘制图片________________ ##
num = min(len(x1), len(x2))
idx = [i for i in range(num)]

points = np.arange(-0.1, 1.1, 0.01)
xs, ys = np.meshgrid(points, points)
z = function(xs, ys)

plt.figure()
plt.contourf(xs, ys, z, 50, alpha = 0.8, cmap="rainbow")
plt.contour(xs, ys, z, 50, colors='black')
plt.scatter(x1, x2, s=60, c="r", marker="x", alpha=1)
plt.plot(x1, x2, color="black", linewidth=1, linestyle="-", label="iter", alpha=0.5)

plt.show()
