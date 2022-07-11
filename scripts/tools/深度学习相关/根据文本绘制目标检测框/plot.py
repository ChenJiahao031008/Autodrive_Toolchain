import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import glob
import os
import cv2
import natsort

image_path = glob.glob(os.path.join("/home/chen/Datasets/rgbd_dataset_freiburg3_long_office_household/rgb/","*.png"))
label_path = glob.glob(os.path.join("/home/chen/lidar_camera/sort-cpp/data/result/","*.txt"))
image_path.sort()
label_path = natsort.natsorted(label_path)

if (len(image_path) != len(label_path)):
	print("[ERRO] Math Failed!")
	print("[ERRO]", len(image_path), " != ", len(label_path))
	exit()
for i in range(len(image_path)):
	img = cv2.imread(image_path[i])
	h,w,c = img.shape
	with open(label_path[i]) as f:
		for data in f.readlines():
			data1 = data.split()
			# print(data1)
			xl =int(float(data1[1]))
			yl =int(float(data1[2]))
			xr =int(float(data1[1])+float(data1[3]))
			yr =int(float(data1[2])+float(data1[4]))
			cv2.rectangle(img,(xl,yl),(xr,yr),(255,0,0),1)
			cv2.putText(img,data1[7],(xl,yl),1,1,(0,255,0))
	cv2.imshow("show", img)
	cv2.waitKey(1)
	address, _ = os.path.split(image_path[i])
	suffix = "/../new_pic/" + str(i) + ".png"
	targe = address + suffix
	print("[INFO] Targe is :",targe)
	cv2.imwrite(targe, img)




