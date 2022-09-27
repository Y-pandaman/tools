# coding=utf-8
import os


class BatchRename():
    def __init__(self):
        self.path1 = '/home/bdca/dataset/new_stereo_imags/source/outdoor1/cam0'  # 文件夹路径
        self.path2 = '/home/bdca/dataset/new_stereo_imags/source/outdoor1/cam1'

    def rename(self):
        filelist = os.listdir(self.path1)  # 获取文件路径
        total_num = len(filelist)  # 获取文件长度（个数）
        i = 1  # 表示文件的命名是从1开始的
        for item in filelist:
            if item.endswith('.png'):  # 初始的图片的格式为png格式的
                src1 = os.path.join(os.path.abspath(self.path1), item)  # 源文件
                src2 = os.path.join(os.path.abspath(self.path2), item)
                # 处理后的格式
                dst1 = os.path.join(os.path.abspath(
                    self.path1), str(i) + '.png')  # 目标文件
                dst2 = os.path.join(os.path.abspath(
                    self.path2), str(i) + '.png')
                # dst = os.path.join(os.path.abspath(
                #     self.path), '0000' + format(str(i), '0>3s') + '.png')    # 这种情况下的命名格式为0000001.jpg形式
                try:
                    os.rename(src1, dst1)
                    os.rename(src2, dst2)
                    print('converting %s to %s ...' % (src1, dst1))
                    print('converting %s to %s ...' % (src2, dst2))
                    i = i + 1
                except:
                    continue
        print('一共重命名了%d张图片' % total_num)


if __name__ == '__main__':
    demo = BatchRename()
    demo.rename()
