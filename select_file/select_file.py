# coding=utf-8
import os
import random
import shutil


def moveFile(fileDir1, fileDir2):
    pathDir = os.listdir(fileDir1)  # 取图片的原始路径
    filenumber = len(pathDir)
#     print(filenumber)  # 打印文件数量
    rate = 0.01  # 自定义抽取图片的比例
#     picknumber = int(filenumber*rate)  # 按照rate比例从文件夹中取一定数量图片
    picknumber = 50  # 自定义选取图片数量
    sample = random.sample(pathDir, picknumber)  # 随机选取picknumber数量的样本图片
#     print(sample) # 打印文件名
    for name in sample:
        # shutil.move(fileDir1 + name, tarDir1 + name)  # 移动图片
        # shutil.move(fileDir2 + name, tarDir2 + name)
        shutil.copy(fileDir1 + name, tarDir1 + name)  # 复制图片
        shutil.copy(fileDir2 + name, tarDir2 + name)
    return


if __name__ == '__main__':
    fileDir1 = "/home/bdca/dataset/new_stereo_imags/outdoor1/cam0/"  # 源图片文件夹路径
    fileDir2 = "/home/bdca/dataset/new_stereo_imags/outdoor1/cam1/"
    tarDir1 = "/home/bdca/dataset/new_stereo_imags/source/outdoor1/cam0/"  # 移动到新的文件夹路径
    tarDir2 = "/home/bdca/dataset/new_stereo_imags/source/outdoor1/cam1/"
    moveFile(fileDir1, fileDir2)
