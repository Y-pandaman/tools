# coding=utf-8
import os
path = '/media/bdca/data3/yolo_new/train'  # 替换你的文件夹
path_result = path + "/result_final"
listdir = os.listdir(path)

# VIO
# num = {
#     "5": '6',
#     "6": '4',
#     "7": '1',
#     "8": '2', 
#     "9": '3'
# }
# VOC
# num = {
#     "21": '5',
#     "23": '7',
#     "25": '4',
#     "26": '1', 
#     "33": '2',
#     "34": '0',
#     "38": '8'
# }
#SDC
# num = {
#     "20": '5',
#     "21": '1',
#     "22": '0',
#     "30": '3'
# }
#COCO
# num = {
#     "109": '5',
#     "111": '7',
#     "116": '4',
#     "118": '1',
#     "143": '2',
#     "148": '0',
#     "173": '8',
#     "174": '3'
# }
#construction vehicle
num = {
    "0": '0',
    "1": '1',
    "2": '1',
    "3": '1',
    "4": '1',
    "5": '1',
    "6": '1',
    "7": '1',
    "8": '1',
    "9": '2',
    "10": '2',
    "11": '2',
    "12": '2',
    "13": '2',
    "14": '2'
}

try:
    os.mkdir(path_result)
except FileExistsError:
    pass
except:
    print('已经改写，若重改请删除结果文件夹')
for f_name in listdir:
    path_filename = path + "/" + f_name
    print(path_filename)
    with open(path_filename) as txt:
        os.mknod(path_result+'/' + f_name)
        for i in txt.readlines():
            a = i.split(' ')
            c = num.get(str(a[0])) + ' ' + a[1] + ' ' + a[2] + ' ' + a[3] + ' ' + a[4]
            with open(path_result + '/' + f_name, 'a') as txt_result:
                txt_result.write(c)
# os.system(path_result)
