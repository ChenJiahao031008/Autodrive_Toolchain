#!/usr/bin/python
# -*- coding: UTF-8 -*- 

import glob
import os

image_path = glob.glob(os.path.join("./test_image","*.jpg"))
label_path = glob.glob(os.path.join("./test_label","*.txt"))
alignment_txt = "test.txt"

image_path.sort()
label_path.sort()

assert (len(image_path)==len(label_path))

with open(alignment_txt, 'w') as f:
	for i in range(len(image_path)):
		ls = []
		ls.append(image_path[i])
		ls.append(' ')
		ls.append(label_path[i])
		if (i!=len(image_path)-1):
			ls.append('\n')
		f.writelines(ls)