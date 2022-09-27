# coding=utf-8
import os
path = '/home/psdz/桌面/下载/COCO/train/labels'  # 替换你的文件夹
path_result = path + "/result"
listdir = os.listdir(path)
# list = [0, 2, 3, 4, 7, 8, 9, 10, 11, 12, 15, 16, 17, 19] # VOC
# list = [3, 4, 5, 6, 7, 8, 9] # SDC
list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 13, 14, 15, 17, 19, 20, 
        21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 
        38, 39, 40, 41, 42, 44, 45, 46, 47, 49, 50, 51, 52, 53, 54, 55, 56, 
        57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 75, 
        76, 77, 78, 79]  # COCO

try:
    os.mkdir(path_result)
except FileExistsError:
    pass
except:
    print('已经改写，若重改请删除结果文件夹')
for f_name in listdir:
    path_filename = path + "/" + f_name
    # print(path_filename)
    with open(path_filename) as txt:
        os.mknod(path_result + '/' + f_name)
        for i in txt.readlines():
            a = i.split(' ')
            # b = str(int(a[0])+1)
            # if (int(a[0]) == 6 or int(a[0]) == 5): # 刪除第1列數值爲5或6的行
            if int(a[0]) in list:
                continue
            c = a[0] + ' ' + a[1] + ' ' + a[2] + ' ' + a[3] + ' ' + a[4]
            with open(path_result + '/' + f_name, 'a') as txt_result:
                txt_result.write(c)
# os.system(path_result)
