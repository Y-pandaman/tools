# coding=utf-8
import os
path = '/home/psdz/桌面/下载/COCO/train/labels/result'  # 替换你的文件夹
path_result = path + "/result_mid"
listdir = os.listdir(path)
try:
    os.mkdir(path_result)
except FileExistsError:
    pass
except:
    print('已经改写，若重改请删除结果文件夹')
for f_name in listdir:
    path_filename = path + "/"+f_name
    print(path_filename)
    with open(path_filename) as txt:
        os.mknod(path_result + '/' + f_name)
        for i in txt.readlines():
            a = i.split(' ')
            b = str(int(a[0]) + 100)
            c = b + ' ' + a[1] + ' ' + a[2] + ' ' + a[3] + ' ' + a[4]
            with open(path_result + '/'+f_name, 'a') as txt_result:
                txt_result.write(c)
