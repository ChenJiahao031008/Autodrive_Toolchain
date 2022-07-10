import random
import glob
import os
import shutil
import natsort

# —————————————————————————————————————————————————————— #
# 参数配置区域，请修改下列参数
src_image_path = "./test_image"
src_label_path = "./test_label"
dst_image_path = "./new_dataset/image/"
dst_label_path = "./new_dataset/label/"
# —————————————————————————————————————————————————————— #

image_path = glob.glob(os.path.join(src_image_path,"*.jpg"))
label_path = glob.glob(os.path.join(src_label_path,"*.txt"))


image_path = natsort.natsorted(image_path)
label_path = natsort.natsorted(label_path)

## 以相同的顺序打乱
randnum = random.randint(0,100)
random.seed(randnum)
random.shuffle(image_path)
random.seed(randnum)
random.shuffle(label_path)

print("[INFO] new sequence: \n")
print(image_path)
print(label_path)
print("=======================")

assert (len(image_path)==len(label_path))

print("[WARNING] rm rf ./new_dataset/! ")
print("=======================")
os.system("rm -rf ./new_dataset/")
os.system("mkdir -p ./new_dataset/image/")
os.system("mkdir -p ./new_dataset/label/")

for id_number in range(len(image_path)):
	prefix = "new_" + str(id_number)
	image_name = prefix + ".jpg"
	label_name = prefix + ".txt"

	source = image_path[id_number]
	target = dst_image_path + image_name;
	print("[INFO] current target is:", target)

	shutil.copy(source, target)

	source = label_path[id_number]
	target = dst_label_path + label_name;
	print("[INFO] current target is:", target)

	shutil.copy(source, target)


