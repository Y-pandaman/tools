#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

int it_num = 100;        // 迭代次数
float f = 1000.0;        // 相机焦距
int flag_it = 0;         // 是否进行迭代
int cylinder_method = 1; // 投影方法
int stitch_method = 1;   // 拼接方法
int offset_x = 0;        // 横向偏移
int offset_y = 0;        // 纵向偏移
int is_resize = 0;       // 是否进行缩放
int re_cols = 0;         // 缩放后的列数
int re_rows = 0;         // 缩放后的行数
int is_cal_offset = 0;   // 是否计算偏移
int flag_mul = 1;        // 是否进行多图拼接
int offset_x_1 = 0;      // 第1次横向偏移
int offset_y_1 = 0;      // 第2次纵向偏移
int offset_x_2 = 0;      // 第2次横向偏移
int offset_y_2 = 0;      // 第2次纵向偏移
int offset_x_3 = 0;      // 第3次横向偏移
int offset_y_3 = 0;      // 第3次纵向偏移
int offset_x_4 = 0;      // 第4次横向偏移
int offset_y_4 = 0;      // 第4次纵向偏移
int offset_x_5 = 0;      // 第5次横向偏移
int offset_y_5 = 0;      // 第5次纵向偏移
int offset_x_6 = 0;      // 第6次横向偏移
int offset_y_6 = 0;      // 第6次纵向偏移
int offset_x_7 = 0;      // 第7次横向偏移
int offset_y_7 = 0;      // 第7次纵向偏移
int offset_x_8 = 0;      // 第8次横向偏移
int offset_y_8 = 0;      // 第8次纵向偏移
int offset_x_9 = 0;      // 第9次横向偏移
int offset_y_9 = 0;      // 第9次纵向偏移
int offset_x_10 = 0;     // 第10次横向偏移
int offset_y_10 = 0;     // 第10次纵向偏移
int show_temp = 0;       // 是否显示模板图像

std::vector<cv::Point2i> offsets;
int time_ = 1;

int posTrackBar = 0; // trackbar的值
int maxValue = 255;  // trackbar的最大值
#endif