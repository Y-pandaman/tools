import os
def delete_file(dir, size):
	fl = os.listdir(dir)
	print(len(fl))
	i = 0
	for file in fl:
		fsize = os.path.getsize(f'/public/experiments/ypt/xugong/datasets/yolo/train/{file}')
		name = os.path.splitext(file)
		png = name[0] + ".jpg"
		if fsize == size:
			print(file)
			# print(name[0])
			print(i)
			i = i + 1
			os.remove(f'/public/experiments/ypt/xugong/datasets/yolo/train/{file}')
			if os.path.exists(f'/public/experiments/ypt/xugong/datasets/yolo/train/{png}'):
				os.remove(f'/public/experiments/ypt/xugong/datasets/yolo/train/{png}')

if __name__ == "__main__":
	print("start")
	delete_file("/public/experiments/ypt/xugong/datasets/yolo/train/", 0)