#include "Pow.h"
#include "atan.h"
#include "config.h"
#include <emmintrin.h>
#include <iostream>
#include <omp.h>
#include <openacc.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pmmintrin.h>
#include <thread>
#include <time.h>
#include <xmmintrin.h>

cv::Mat cylindrical_projection(cv::Mat img, int cylinder_method);
cv::Mat stitch_image(cv::Mat stitch, cv::Mat img1, cv::Mat img2, cv::Point2i a,
                     int stitch_method);
void get_param();

/*柱面投影函数
 * @param img:输入图像
 * @param f:焦距
 * @return:柱面投影后的图像
 */

// 数组
cv::Mat cylinder1(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan(0.5 * img.cols / f);
  rowNum = img.rows;

  // Mat imgOut = Mat::zeros(rowNum, colNum, CV_8UC3);
  cv::Mat result(rowNum, colNum, img.type());

  int x1(0), y1(0);
  for (int i = 0; i < img.cols; i++) {
    for (int j = 0; j < img.rows; j++) {
      x1 = f * atan((i - 0.5 * img.rows) / f) + f * atan(0.5 * img.rows / f);
      y1 = f * (j - 0.5 * img.rows) /
               sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
           0.5 * img.rows;
      if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
        result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
      }
    }
  }
  return result;
}

// 指针
cv::Mat cylinder2(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan(0.5 * img.cols / f);
  rowNum = img.rows;

  // Mat imgOut = Mat::zeros(rowNum, colNum, CV_8UC3);
  cv::Mat imgOut(rowNum, colNum, img.type());
  uchar *im1 = img.data;
  uchar *im2 = imgOut.ptr<uchar>(0);
  // int dim = img.channels();

  int x1(0), y1(0);
  for (int i = 0; i < img.cols; i++) {
    for (int j = 0; j < img.rows; j++) {
      x1 = f * atan((i - 0.5 * img.rows) / f) + f * atan(0.5 * img.rows / f);
      y1 = f * (j - 0.5 * img.rows) /
               sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
           0.5 * img.rows;
      if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
        im2[y1 * colNum * 3 + x1 * 3] = im1[j * img.cols * 3 + i * 3];
        im2[y1 * colNum * 3 + x1 * 3 + 1] = im1[j * img.cols * 3 + i * 3 + 1];
        im2[y1 * colNum * 3 + x1 * 3 + 2] = im1[j * img.cols * 3 + i * 3 + 2];
      }
    }
  }
  return imgOut;
}

// 中心投影
cv::Mat cylinder3(cv::Mat img, int f) {
  cv::Mat imgOut(img.rows, img.cols, img.type());

  int width = img.cols, height = img.rows;
  double x, y;
  int drcpoint;
  for (int hnum = 0; hnum < height; hnum++) {
    for (int wnum = 0; wnum < width; wnum++) {
      double k = f / sqrt(f * f + (wnum - width / 2) * (wnum - width / 2));
      x = (wnum - width / 2) / k + width / 2;
      y = (hnum - height / 2) / k + height / 2;
      if (0 < x && x < width && 0 < y && y < height) {
        imgOut.at<cv::Vec3b>(hnum, wnum) = img.at<cv::Vec3b>(int(y), int(x));
      }
    }
  }
  return imgOut;
}

// parallel_for_ 单循环
cv::Mat cylinder4(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan(0.5 * img.cols / f);
  rowNum = img.rows;

  cv::Mat result(rowNum, colNum, img.type());

  cv::parallel_for_(
      cv::Range(0, img.cols * img.rows), [&](const cv::Range &range) {
        for (int k = range.start; k < range.end; k++) {
          int i = k % img.cols;
          int j = k / img.cols;
          auto x1 =
              f * atan((i - 0.5 * img.rows) / f) + f * atan(0.5 * img.rows / f);
          auto y1 = f * (j - 0.5 * img.rows) /
                        sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
                    0.5 * img.rows;
          if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
            result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
          }
        }
      });
  return result;
}

// parallel_for_ 双循环
cv::Mat cylinder5(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan(0.5 * img.cols / f);
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  cv::parallel_for_(cv::Range(0, img.cols), [&](const cv::Range &range_col) {
    for (int i = range_col.start; i < range_col.end; i++) {
      cv::parallel_for_(
          cv::Range(0, img.rows), [&](const cv::Range &range_row) {
            for (int j = range_row.start; j < range_row.end; j++) {
              auto x1 = f * atan((i - 0.5 * img.rows) / f) +
                        f * atan(0.5 * img.rows / f);
              auto y1 = f * (j - 0.5 * img.rows) /
                            sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
                        0.5 * img.rows;

              if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
                result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
              }
            }
          });
    }
  });

  return result;
}

// openMP 加速
cv::Mat cylinder6(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan(0.5 * img.cols / f);
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  int x1(0), y1(0);
  omp_set_num_threads(8);
#pragma omp parallel for
  for (int i = 0; i < img.cols; i++) {
    for (int j = 0; j < img.rows; j++) {
      x1 = f * atan((i - 0.5 * img.rows) / f) + f * atan(0.5 * img.rows / f);
      y1 = f * (j - 0.5 * img.rows) /
               sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
           0.5 * img.rows;
      if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
        result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
      }
    }
  }
  return result;
}

// openMP + 单循环
cv::Mat cylinder7(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan(0.5 * img.cols / f);
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  omp_set_num_threads(8);
#pragma omp parallel for
  for (int k = 0; k < img.cols * img.rows; k++) {
    int i = k % img.cols;
    int j = k / img.cols;
    auto x1 = f * atan((i - 0.5 * img.rows) / f) + f * atan(0.5 * img.rows / f);
    auto y1 = f * (j - 0.5 * img.rows) /
                  sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
              0.5 * img.rows;
    if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
      result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
    }
  }
  return result;
}

// parallel_for_ 单循环 + atan查表
cv::Mat cylinder8(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan_single(0.5 * img.cols / f);
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  cv::parallel_for_(
      cv::Range(0, img.cols * img.rows), [&](const cv::Range &range) {
        for (int k = range.start; k < range.end; k++) {
          int i = k % img.cols;
          int j = k / img.cols;
          auto x1 = f * atan_single((i - 0.5 * img.rows) / f) +
                    f * atan_single(0.5 * img.rows / f);
          auto y1 = f * (j - 0.5 * img.rows) /
                        sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
                    0.5 * img.rows;
          if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
            result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
          }
        }
      });
  return result;
}

// openMP 单循环 + atan查表
cv::Mat cylinder9(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan_single(0.5 * img.cols / f);
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  omp_set_num_threads(8);
#pragma omp parallel for
  for (int k = 0; k < img.rows * img.cols; k++) {
    int i = k % img.cols;
    int j = k / img.cols;
    auto x1 = f * atan_single((i - 0.5 * img.rows) / f) +
              f * atan_single(0.5 * img.rows / f);
    auto y1 = f * (j - 0.5 * img.rows) /
                  sqrt(pow(i - 0.5 * img.cols, 2) + pow(f, 2)) +
              0.5 * img.rows;
    if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
      result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
    }
  }
  return result;
}

// parallel_for_ 单循环 + 先定义变量 + atan查表
cv::Mat cylinder10(cv::Mat img, int f) {
  int colNum, rowNum;
  colNum = 2 * f * atan_single(0.5 * img.cols / f);
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());
  float pow_f = pow(f, 2);
  float half_row = 0.5 * img.rows;
  float half_col = 0.5 * img.cols;
  float img_size = img.cols * img.rows;

  cv::parallel_for_(cv::Range(0, img_size), [&](const cv::Range &range) {
    for (int k = range.start; k < range.end; k++) {
      int i = k % img.cols;
      int j = k / img.cols;
      auto x1 =
          f * atan_single((i - half_row) / f) + f * atan_single(half_row / f);
      auto y1 =
          f * (j - half_row) / sqrt(pow(i - half_col, 2) + pow_f) + half_row;
      if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
        result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
      }
    }
  });
  return result;
}

// parallel_for_ 单循环 + 先定义变量 + pow加速 + atan查表
cv::Mat cylinder11(cv::Mat img, int f) {
  int colNum, rowNum;
  // int pow_f = power_fun(f, 2);
  int pow_f = Sqr(f);
  float half_row = img.rows >> 1;
  float half_col = img.cols >> 1;
  int img_size = img.cols * img.rows;
  colNum = (int(f * atan_single(half_col / f))) << 1;
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  cv::parallel_for_(cv::Range(0, img_size), [&](const cv::Range &range) {
    for (int k = range.end - 1; k >= range.start; k--) {
      int i = k % img.cols;
      int j = k / img.cols;

      auto x1 =
          f * atan_single((i - half_row) / f) + f * atan_single(half_row / f);
      auto y1 = f * (j - half_row) / sqrt(power_fun(i - half_col, 2) + pow_f) +
                half_row;
      // auto y1 =
      //     f * (j - half_row) / sqrt(Sqr(i - half_col) + pow_f) + half_row;
      if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
        result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
      }
    }
  });
  return result;
}

// openacc + 单循环 + 先定义变量 + pow加速 + atan查表
cv::Mat cylinder12(cv::Mat img, int f) {
  int colNum, rowNum;
  int pow_f = Sqr(f);
  float half_row = img.rows >> 1;
  float half_col = img.cols >> 1;
  int img_size = img.cols * img.rows;
  colNum = (int(f * atan_single(half_col / f))) << 1;
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  // cv::parallel_for_(cv::Range(0, img_size), [&](const cv::Range &range) {
  //   for (int k = range.end - 1; k >= range.start; k--) {
#pragma acc parallel
#pragma acc loop independent
  for (int k = img_size - 1; k >= 0; k--) {
    int i = k % img.cols;
    int j = k / img.cols;

    auto x1 =
        f * atan_single((i - half_row) / f) + f * atan_single(half_row / f);
    auto y1 = f * (j - half_row) / sqrt(power_fun(i - half_col, 2) + pow_f) +
              half_row;
    if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
      result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
    }
  }
  // });
  return result;
}

// parallel_for_ 单循环 + 先定义变量 + pow加速 + atan查表 + 去黑边
cv::Mat cylinder13(cv::Mat img, int f) {
  int colNum, rowNum;
  // int pow_f = power_fun(f, 2);
  int pow_f = Sqr(f);
  float half_row = img.rows >> 1;
  float half_col = img.cols >> 1;
  int img_size = img.cols * img.rows;
  colNum = (int(f * atan_single(half_col / f))) << 1;
  rowNum = img.rows;
  cv::Mat result(rowNum, colNum, img.type());

  cv::parallel_for_(cv::Range(0, img_size), [&](const cv::Range &range) {
    for (int k = range.end - 1; k >= range.start; k--) {
      int i = k % img.cols;
      int j = k / img.cols;
      auto x1 =
          f * atan_single((i - half_row) / f) + f * atan_single(half_row / f);
      auto y1 = f * (j - half_row) / sqrt(power_fun(i - half_col, 2) + pow_f) +
                half_row;
      if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
        result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
      }
    }
  });

  cv::Mat threshold_img, gray;
  cv::cvtColor(result, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, threshold_img, 0, 255, cv::THRESH_BINARY);
  cv::Rect rect = cv::boundingRect(threshold_img);
  return result(cv::Range(rect.y, rect.y + rect.height),
                cv::Range(rect.x, rect.x + rect.width));
  // return result;
}

// // parallel_for_ 单循环 + 先定义变量 + pow加速 + atan查表 + sse
// cv::Mat cylinder14(cv::Mat img, int f) {
//   int colNum, rowNum;
//   // int pow_f = power_fun(f, 2);
//   int pow_f = Sqr(f);
//   float half_row = img.rows >> 1;
//   float half_col = img.cols >> 1;
//   int img_size = img.cols * img.rows;
//   colNum = (int(f * atan_single(half_col / f))) << 1;
//   rowNum = img.rows;
//   cv::Mat result(rowNum, colNum, img.type());

//   cv::parallel_for_(cv::Range(0, img_size), [&](const cv::Range &range) {
//     for (int k = range.end - 1; k >= range.start; k--) {
//       int i = k % img.cols;
//       int j = k / img.cols;
//       int x1, y1;
//       // auto x1 =
//       //     f * atan_single((i - half_row) / f) + f * atan_single(half_row /
//       //     f);
//       // auto y1 = f * (j - half_row) / sqrt(power_fun(i - half_col, 2) +
//       pow_f)
//       // +
//       //           half_row;
//       _m128 v1 =
//           _mm_loadu_ps(f * atan_single((i - half_row) / f));
//       _m128 v2 = f * _mm_set_ps1(atan_single(half_row / f));
//       _m128 v3 = f * _mm_set_ps1(j - half_row) /
//                  _mm_set_ps1(sqrt(power_fun(i - half_col, 2)));
//       _m128i vdata1 = _mm_add_ps(v1, v2);
//       _m128i vdata2 = _mm_mul_ps(v3, _mm_set_ps1(pow_f));
//       _mm_store_ss(x1, &vdata1);
//       _mm_store_ss(y1, &vdata2);
//       if (x1 >= 0 && x1 < colNum && y1 >= 0 && y1 < rowNum) {
//         result.at<cv::Vec3b>(y1, x1) = img.at<cv::Vec3b>(j, i);
//       }
//     }
//   });
//   return result;
// }

// 模板匹配
// 通过模板匹配的方法求取平移变换参数
/* 求平移量
 * @param:
 * 		img1: 图像1
 * 		img2: 图像2
 * @return：点类型
 * 		x: 平移量
 * 		y: 平移量
 */

cv::Point2i getOffset(cv::Mat img1, cv::Mat img2) {
  cv::Mat templ(img2,
                cv::Rect(0, 0.4 * img2.rows, 0.2 * img2.cols, 0.2 * img2.rows));
  cv::Mat result(img1.cols - templ.cols + 1, img1.rows - templ.rows + 1,
                 img1.type());
  matchTemplate(img1, templ, result, CV_TM_CCORR_NORMED);
  normalize(result, result, 0, 1, cv::NORM_MINMAX, -1,
            cv::Mat()); // 对匹配结果归一化
  double minVal;
  double maxVal;
  cv::Point minLoc;
  cv::Point maxLoc;
  cv::Point matchLoc;
  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc,
            cv::Mat()); // 在一个数组中找到全局最小值和最大值
  matchLoc = maxLoc;    // 获得最佳匹配位置
  // 显示模板在图像中的匹配位置
  if (show_temp) {
    cv::rectangle(img1, matchLoc,
                  cv::Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows),
                  cv::Scalar(0, 0, 255), 2, 8, 0);
    cv::imwrite("../result/templ_ori" + std::to_string(time_++) + ".jpg", img1);
  }

  int dx = matchLoc.x;
  int dy = matchLoc.y - 0.4 * img2.rows; // 右图相对左图的平移量
  cv::Point2i a(dx, dy);
  return a;
}

// 线性融合
// 采用渐入渐出融合，其实就是在重叠区域，对两幅图像的像素，线性地分配权值。
// 公式：img1=d∗img1+(1−d)∗img2；
// 其中img1为融合后的图像，img2和img2为待拼接的两幅图像。
// d为重叠区域中某个像素点到边界的距离。
/*
 * @param img1 待拼接的图像1
 * @param img2 待拼接的图像2
 * @param a 偏移量
 * @return 拼接后的图像
 */
// 1
cv::Mat linearStitch(cv::Mat img1, cv::Mat img2, cv::Point2i a) {
  int d = img1.cols - a.x;       // 过渡区域的宽度
  int ms = img1.rows - abs(a.y); // 拼接图行数
  int ns = img1.cols + a.x;      // 拼接图列数
  // Mat stitch = Mat::zeros(ms, ns, CV_8UC3);
  cv::Mat stitch(ms, ns, img1.type());
  // int depth = img1.channels();

  if (a.y >= 0) {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(a.y, img1.rows), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, a.x, ms));
    img2(cv::Range(0, ms), cv::Range(d, img2.cols)).copyTo(roi2);
    for (int i = 0; i < ms; i++) {
      for (int j = a.x; j < img1.cols; j++) {
        stitch.at<cv::Vec3b>(i, j)[0] = uchar(
            (img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i + a.y, j)[0] +
            (j - a.x) / float(d) * img2.at<cv::Vec3b>(i, j - a.x)[0]);
        stitch.at<cv::Vec3b>(i, j)[1] = uchar(
            (img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i + a.y, j)[1] +
            (j - a.x) / float(d) * img2.at<cv::Vec3b>(i, j - a.x)[1]);
        stitch.at<cv::Vec3b>(i, j)[2] = uchar(
            (img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i + a.y, j)[2] +
            (j - a.x) / float(d) * img2.at<cv::Vec3b>(i, j - a.x)[2]);
      }
    }
  } else {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(0, ms), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, a.x, ms));
    img2(cv::Range(-a.y, img1.rows), cv::Range(d, img2.cols)).copyTo(roi2);
    for (int i = 0; i < ms; i++) {
      for (int j = a.x; j < img1.cols; j++) {
        stitch.at<cv::Vec3b>(i, j)[0] =
            uchar((img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i, j)[0] +
                  (j - a.x) / float(d) *
                      img2.at<cv::Vec3b>(i + abs(a.y), j - a.x)[0]);
        stitch.at<cv::Vec3b>(i, j)[1] =
            uchar((img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i, j)[1] +
                  (j - a.x) / float(d) *
                      img2.at<cv::Vec3b>(i + abs(a.y), j - a.x)[1]);
        stitch.at<cv::Vec3b>(i, j)[2] =
            uchar((img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i, j)[2] +
                  (j - a.x) / float(d) *
                      img2.at<cv::Vec3b>(i + abs(a.y), j - a.x)[2]);
      }
    }
  }

  return stitch;
}

// 2 parallel_for_并行
cv::Mat linearStitch_parallel_for_(cv::Mat img1, cv::Mat img2, cv::Point2i a) {
  float d = img1.cols - a.x;     // 过渡区域的宽度
  int ms = img1.rows - abs(a.y); // 拼接图行数
  int ns = img1.cols + a.x;      // 拼接图列数
  // Mat stitch = Mat::zeros(ms, ns, CV_8UC3);
  cv::Mat stitch(ms, ns, img1.type());
  // int depth = img1.channels();
  int abs_a_y = abs(a.y);

  if (a.y >= 0) {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(a.y, img1.rows), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, a.x, ms));
    img2(cv::Range(0, ms), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::parallel_for_(cv::Range(0, ms), [&](const cv::Range &range_col) {
      for (int i = range_col.start; i < range_col.end; i++) {
        cv::parallel_for_(
            cv::Range(a.x, img1.cols), [&](const cv::Range &range_row) {
              for (int j = range_row.start; j < range_row.end; j++) {
                stitch.at<cv::Vec3b>(i, j) =
                    (img1.cols - j) / d * img1.at<cv::Vec3b>(i + a.y, j) +
                    (j - a.x) / d * img2.at<cv::Vec3b>(i, j - a.x);
              }
            });
      }
    });
  } else {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(0, ms), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, a.x, ms));
    img2(cv::Range(-a.y, img1.rows), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::parallel_for_(cv::Range(0, ms), [&](const cv::Range &range_col) {
      for (int i = range_col.start; i < range_col.end; i++) {
        cv::parallel_for_(
            cv::Range(a.x, img1.cols), [&](const cv::Range &range_row) {
              for (int j = range_row.start; j < range_row.end; j++) {
                stitch.at<cv::Vec3b>(i, j) =
                    (img1.cols - j) / d * img1.at<cv::Vec3b>(i, j) +
                    (j - a.x) / d * img2.at<cv::Vec3b>(i + abs_a_y, j - a.x);
              }
            });
      }
    });
  }

  return stitch;
}

// 3 openMP并行
cv::Mat linearStitch_openMP(cv::Mat img1, cv::Mat img2, cv::Point2i a) {
  int d = img1.cols - a.x;       // 过渡区域的宽度
  int ms = img1.rows - abs(a.y); // 拼接图行数
  int ns = img1.cols + a.x;      // 拼接图列数
  cv::Mat stitch(ms, ns, img1.type());

  if (a.y >= 0) {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(a.y, img1.rows), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, a.x, ms));
    img2(cv::Range(0, ms), cv::Range(d, img2.cols)).copyTo(roi2);

    omp_set_num_threads(8);
#pragma omp parallel for
    for (int i = 0; i < ms; i++) {
      for (int j = a.x; j < img1.cols; j++) {
        stitch.at<cv::Vec3b>(i, j) =
            (img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i + a.y, j) +
            (j - a.x) / float(d) * img2.at<cv::Vec3b>(i, j - a.x);
      }
    }
  } else {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(0, ms), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, a.x, ms));
    img2(cv::Range(-a.y, img1.rows), cv::Range(d, img2.cols)).copyTo(roi2);

    omp_set_num_threads(8);
#pragma omp parallel for
    for (int i = 0; i < ms; i++) {
      for (int j = a.x; j < img1.cols; j++) {
        stitch.at<cv::Vec3b>(i, j) =
            (img1.cols - j) / float(d) * img1.at<cv::Vec3b>(i, j) +
            (j - a.x) / float(d) * img2.at<cv::Vec3b>(i + abs(a.y), j - a.x);
      }
    }
  }

  return stitch;
}

// 4
cv::Mat linearStitch_parallel_for_mul(cv::Mat img1, cv::Mat img2,
                                      cv::Point2i a) {
  float d = img1.cols - a.x;     // 过渡区域的宽度
  int ms = img1.rows - abs(a.y); // 拼接图行数
  int ns = img2.cols + a.x;      // 拼接图列数
  cv::Mat stitch(ms, ns, img1.type());
  int abs_a_y = abs(a.y);

  if (a.y >= 0) {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(a.y, img1.rows), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, img2.cols - d, ms));
    img2(cv::Range(0, ms), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::parallel_for_(cv::Range(0, ms), [&](const cv::Range &range_col) {
      for (int i = range_col.start; i < range_col.end; i++) {
        cv::parallel_for_(
            cv::Range(a.x, img1.cols), [&](const cv::Range &range_row) {
              for (int j = range_row.start; j < range_row.end; j++) {
                stitch.at<cv::Vec3b>(i, j) =
                    (img1.cols - j) / d * img1.at<cv::Vec3b>(i + a.y, j) +
                    (j - a.x) / d * img2.at<cv::Vec3b>(i, j - a.x);
              }
            });
      }
    });
  } else {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(0, ms), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, img2.cols - d, ms));
    img2(cv::Range(-a.y, img2.rows), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::parallel_for_(cv::Range(0, ms), [&](const cv::Range &range_col) {
      for (int i = range_col.start; i < range_col.end; i++) {
        cv::parallel_for_(
            cv::Range(a.x, img1.cols), [&](const cv::Range &range_row) {
              for (int j = range_row.start; j < range_row.end; j++) {
                stitch.at<cv::Vec3b>(i, j) =
                    (img1.cols - j) / d * img1.at<cv::Vec3b>(i, j) +
                    (j - a.x) / d * img2.at<cv::Vec3b>(i + abs_a_y, j - a.x);
              }
            });
      }
    });
  }

  return stitch;
}

// 5
cv::Mat linearStitch_parallel_for_mul_not_cut(cv::Mat img1, cv::Mat img2,
                                              cv::Point2i a) {
  float d = img1.cols - a.x; // 过渡区域的宽度
  // int ms = img1.rows - abs(a.y); // 拼接图行数
  int ms = img1.rows + abs(a.y); // 拼接图行数 高
  int ns = img2.cols + a.x;      // 拼接图列数 宽
  // std::cout << "ms: " << ms << std::endl;
  // std::cout << "ns: " << ns << std::endl;
  // std::cout << "d: " << d << std::endl;
  // std::cout << img1.rows << " " << img1.cols << std::endl;
  // std::cout << img2.rows << " " << img2.cols << std::endl;
  cv::Mat stitch = cv::Mat::zeros(ms, ns, img1.type());
  int abs_a_y = abs(a.y);
  float seam_x = round(img1.cols + d / 2); // 过渡区域的中心点
  int windows_size = 2;

  if (a.y >= 0) {
    // cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, img1.rows));
    // img1(cv::Range(0, img1.rows), cv::Range(0, a.x)).copyTo(roi1);
    // cv::Mat roi2(stitch, cv::Rect(img1.cols, a.y, img2.cols - d, img2.rows));
    // img2(cv::Range(0, img2.rows), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::Mat roi1(stitch, cv::Rect(0, 0, img1.cols, img1.rows));
    img1.copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, a.y, img2.cols - d, img2.rows));
    img2(cv::Range(0, img2.rows), cv::Range(d, img2.cols)).copyTo(roi2);

    cv::parallel_for_(
        cv::Range(a.y, img1.rows), [&](const cv::Range &range_row) {
          for (int i = range_row.start; i < range_row.end; i++) {
            cv::parallel_for_(
                cv::Range(a.x, img1.cols), [&](const cv::Range &range_col) {
                  for (int j = range_col.start; j < range_col.end; j++) {
                    // stitch.at<cv::Vec3b>(i, j) =
                    //     (img1.cols - j) / d * img1.at<cv::Vec3b>(i, j) +
                    //     (j - a.x) / d * img2.at<cv::Vec3b>(i, j - a.x);
                    if (i < (seam_x - windows_size))
                      stitch.at<cv::Vec3b>(i, j) =
                          img2.at<cv::Vec3b>(i, j - a.x);
                    else if (i > (seam_x + windows_size))
                      stitch.at<cv::Vec3b>(i, j) = img1.at<cv::Vec3b>(i, j);
                    else {
                      int ratio =
                          (i - seam_x + windows_size) / (pow(windows_size, 2));
                      stitch.at<cv::Vec3b>(i, j) =
                          ratio * img1.at<cv::Vec3b>(i, j) +
                          (1 - ratio) * img2.at<cv::Vec3b>(i, j - a.x);
                    }
                  }
                });
          }
        });
  } else {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(0, ms), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, img2.cols - d, ms));
    img2(cv::Range(-a.y, img2.rows), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::parallel_for_(cv::Range(0, ms), [&](const cv::Range &range_col) {
      for (int i = range_col.start; i < range_col.end; i++) {
        cv::parallel_for_(
            cv::Range(a.x, img1.cols), [&](const cv::Range &range_row) {
              for (int j = range_row.start; j < range_row.end; j++) {
                stitch.at<cv::Vec3b>(i, j) =
                    (img1.cols - j) / d * img1.at<cv::Vec3b>(i, j) +
                    (j - a.x) / d * img2.at<cv::Vec3b>(i + abs_a_y, j - a.x);
              }
            });
      }
    });
  }

  return stitch;
}

// 6 Linear blend with alpha blending
cv::Mat linearStitch_parallel_for_alpha_blending(cv::Mat img1, cv::Mat img2,
                                                 cv::Point2i a) {
  int d = img1.cols - a.x; // 过渡区域的宽度
  // float seam_x = round(d / 2);   // 过渡区域的中心点
  int ms = img1.rows - abs(a.y);         // 拼接图行数
  int ns = img2.cols + a.x;              // 拼接图列数
  int seam_x = round(img1.cols - d / 2); // 过渡区域的中心点
  cv::Mat stitch(ms, ns, img1.type());
  int abs_a_y = abs(a.y);
  int windows_size = 2;

  if (a.y >= 0) {
    int f1 = 0, f2 = 0, f3 = 0;
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(a.y, img1.rows), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, img2.cols - d, ms));
    img2(cv::Range(0, ms), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::parallel_for_(cv::Range(0, ms), [&](const cv::Range &range_row) {
      for (int i = range_row.start; i < range_row.end; i++) {
        cv::parallel_for_(
            cv::Range(a.x, img1.cols), [&](const cv::Range &range_col) {
              for (int j = range_col.start; j < range_col.end; j++) {
                if (j < (seam_x - windows_size)) {
                  f1++;
                  stitch.at<cv::Vec3b>(i, j) = img1.at<cv::Vec3b>(i + a.y, j);
                } else if (j > (seam_x + windows_size)) {
                  f2++;
                  stitch.at<cv::Vec3b>(i, j) = img2.at<cv::Vec3b>(i, j - a.x);
                } else {
                  f3++;
                  float ratio =
                      (j - seam_x + windows_size) / (pow(windows_size, 2));
                  stitch.at<cv::Vec3b>(i, j) =
                      ratio * img1.at<cv::Vec3b>(i + a.y, j) +
                      (1 - ratio) * img2.at<cv::Vec3b>(i, j - a.x);
                }
              }
            });
      }
    });
    std::cout << "f1: " << f1 << std::endl;
    std::cout << "f2: " << f2 << std::endl;
    std::cout << "f3: " << f3 << std::endl;
  } else {
    cv::Mat roi1(stitch, cv::Rect(0, 0, a.x, ms));
    img1(cv::Range(0, ms), cv::Range(0, a.x)).copyTo(roi1);
    cv::Mat roi2(stitch, cv::Rect(img1.cols, 0, img2.cols - d, ms));
    img2(cv::Range(-a.y, img2.rows), cv::Range(d, img2.cols)).copyTo(roi2);
    cv::parallel_for_(cv::Range(0, ms), [&](const cv::Range &range_row) {
      for (int i = range_row.start; i < range_row.end; i++) {
        cv::parallel_for_(
            cv::Range(a.x, img1.cols), [&](const cv::Range &range_col) {
              for (int j = range_col.start; j < range_col.end; j++) {
                if (j < (seam_x - windows_size))
                  stitch.at<cv::Vec3b>(i, j) =
                      img2.at<cv::Vec3b>(i + abs_a_y, j - a.x);
                else if (j > (seam_x + windows_size))
                  stitch.at<cv::Vec3b>(i, j) = img1.at<cv::Vec3b>(i, j);
                else {
                  int ratio =
                      (j - seam_x + windows_size) / (pow(windows_size, 2));
                  stitch.at<cv::Vec3b>(i, j) =
                      ratio * img1.at<cv::Vec3b>(i, j) +
                      (1 - ratio) * img2.at<cv::Vec3b>(i + abs_a_y, j - a.x);
                }
              }
            });
      }
    });
  }

  return stitch;
}

void show_stitch(cv::Mat img1, cv::Mat img2, int is_resize, float f, int cols,
                 int rows, int cylinder_method, int stitch_method,
                 int is_cal_offset, int offset_x, int offset_y) {
  cv::Mat stitch;

  if (is_resize) {
    cv::resize(img1, img1, cv::Size(cols, rows), cv::INTER_LINEAR);
    cv::resize(img2, img2, cv::Size(cols, rows), cv::INTER_LINEAR);
  }
  imshow("left", img1);
  imshow("right", img2);
  double t = (double)cv::getTickCount();
  //柱形投影
  double t3 = (double)cv::getTickCount();
  img1 = cylindrical_projection(img1, cylinder_method);
  img2 = cylindrical_projection(img2, cylinder_method);
  t3 = ((double)cv::getTickCount() - t3) / cv::getTickFrequency();
  //匹配
  double t1 = (double)cv::getTickCount();

  cv::Point2i a;
  if (is_cal_offset) {
    a = getOffset(img1, img2);
    std::cout << "偏移量：" << a << std::endl;
  } else {
    a = cv::Point2i(offset_x, offset_y);
  }

  t1 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
  //拼接
  double t2 = (double)cv::getTickCount();
  stitch = stitch_image(stitch, img1, img2, a, stitch_method);
  t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

  std::cout << "各阶段耗时：" << std::endl;
  std::cout << "柱面投影：" << 1000.0 * t3 << " ms" << '\n'
            << "模板匹配：" << 1000.0 * t1 << " ms" << '\n'
            << "渐入渐出拼接：" << 1000.0 * t2 << " ms" << std::endl;
  std::cout << "总时间：" << 1000.0 * t << " ms" << std::endl;

  cv::imshow("rectify-left", img1);
  cv::imshow("rectify-right", img2);
  cv::imshow("stitch-result", stitch);
  cv::imwrite("rectify.jpg", img1);
  cv::imwrite("rectify1.jpg", img2);
  cv::imwrite("stitch.jpg", stitch);
  cv::waitKey(0);
}

void cal_avg_time(cv::Mat img1, cv::Mat img2, int is_resize, float f, int cols,
                  int rows, int cylinder_method, int stitch_method,
                  int is_cal_offset, int offset_x, int offset_y, int it_num) {
  double t_0 = 0;
  double t_1 = 0;
  double t_2 = 0;
  double t_3 = 0;
  for (int i = 0; i < it_num; i++) {
    std::cout << "第" << i + 1 << "次测试" << std::endl;
    if (is_resize) {
      cv::resize(img1, img1, cv::Size(cols, rows), cv::INTER_LINEAR);
      cv::resize(img2, img2, cv::Size(cols, rows), cv::INTER_LINEAR);
    }

    cv::Mat stitch;
    double t = (double)cv::getTickCount();
    //柱形投影
    double t3 = (double)cv::getTickCount();
    img1 = cylindrical_projection(img1, cylinder_method);
    img2 = cylindrical_projection(img2, cylinder_method);

    t3 = ((double)cv::getTickCount() - t3) / cv::getTickFrequency();
    //匹配
    double t1 = (double)cv::getTickCount();

    cv::Point2i a;
    if (is_cal_offset) {
      a = getOffset(img1, img2);
      std::cout << "偏移量：" << a << std::endl;
    } else {
      a = cv::Point2i(offset_x, offset_y);
    }

    t1 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
    //拼接
    double t2 = (double)cv::getTickCount();
    stitch = stitch_image(stitch, img1, img2, a, stitch_method);
    t2 = ((double)cv::getTickCount() - t2) / cv::getTickFrequency();

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    t_0 += t;
    t_1 += t1;
    t_2 += t2;
    t_3 += t3;
  }
  std::cout << "各阶段耗时：" << std::endl;
  std::cout << "柱面投影：" << 1000.0 * t_3 / it_num << " ms" << '\n'
            << "模板匹配：" << 1000.0 * t_1 / it_num << " ms" << '\n'
            << "渐入渐出拼接：" << 1000.0 * t_2 / it_num << " ms" << std::endl;
  std::cout << "总时间：" << 1000.0 * t_0 / it_num << " ms" << std::endl;
}

void multi_image_stitch(std::vector<cv::Mat> images, int image_num,
                        int is_cal_offset, int offset_x, int offset_y,
                        int stitch_method) {
  double t = (double)cv::getTickCount();
  cv::Mat stitch, img1, img2;
  stitch = images[0];
  cv::Point2i a;
  for (int i = 0; i < image_num - 1; i++) {
    std::cout << "第" << i + 1 << "次拼接" << std::endl;
    img1 = stitch;
    img2 = images[i + 1];

    //匹配
    if (is_cal_offset) {
      a = getOffset(img1, img2);
      std::cout << "图片" << i + 1 << "和" << i + 2 << "的偏移量：" << a
                << std::endl;
    } else {
      a = offsets[i];
      std::cout << "a：" << a << std::endl;
    }

    //拼接
    stitch = stitch_image(stitch, img1, img2, a, stitch_method);

    cv::imwrite("../result/stitch_" + std::to_string(i + 1) + ".jpg", stitch);
  }
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << "总耗时：" << 1000.0 * t << " ms" << std::endl;
  cv::imshow("result", stitch);
  cv::waitKey(0);
}

void get_param() {
  cv::FileStorage fs("../config/config.yaml", cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Failed to open settings file" << std::endl;
    return;
  }
  it_num = fs["it_num"];                   // 迭代次数
  f = fs["f"];                             // 相机焦距
  flag_it = fs["flag_it"];                 // 是否进行迭代
  cylinder_method = fs["cylinder_method"]; // 投影方法
  stitch_method = fs["stitch_method"];     // 拼接方法
  offset_x = fs["offset_x"];               // 横向偏移
  offset_y = fs["offset_y"];               // 纵向偏移
  is_resize = fs["resize"];                // 是否进行缩放
  re_cols = fs["resize_cols"];             // 缩放后的列数
  re_rows = fs["resize_rows"];             // 缩放后的行数
  is_cal_offset = fs["cal_offset"];        // 是否计算偏移
  flag_mul = fs["flag_mul"];               // 是否进行多图拼接
  offset_x_1 = fs["offset_x_1"];           // 第1次横向偏移
  offset_y_1 = fs["offset_y_1"];           // 第1次纵向偏移
  offset_x_2 = fs["offset_x_2"];           // 第2次横向偏移
  offset_y_2 = fs["offset_y_2"];           // 第2次纵向偏移
  offset_x_3 = fs["offset_x_3"];           // 第3次横向偏移
  offset_y_3 = fs["offset_y_3"];           // 第3次纵向偏移
  offset_x_4 = fs["offset_x_4"];           // 第4次横向偏移
  offset_y_4 = fs["offset_y_4"];           // 第4次纵向偏移
  offset_x_5 = fs["offset_x_5"];           // 第5次横向偏移
  offset_y_5 = fs["offset_y_5"];           // 第5次纵向偏移
  offset_x_6 = fs["offset_x_6"];           // 第6次横向偏移
  offset_y_6 = fs["offset_y_6"];           // 第6次纵向偏移
  offset_x_7 = fs["offset_x_7"];           // 第7次横向偏移
  offset_y_7 = fs["offset_y_7"];           // 第7次纵向偏移
  offset_x_8 = fs["offset_x_8"];           // 第8次横向偏移
  offset_y_8 = fs["offset_y_8"];           // 第8次纵向偏移
  offset_x_9 = fs["offset_x_9"];           // 第9次横向偏移
  offset_y_9 = fs["offset_y_9"];           // 第9次纵向偏移
  offset_x_10 = fs["offset_x_10"];         // 第10次横向偏移
  offset_y_10 = fs["offset_y_10"];         // 第10次纵向偏移
  show_temp = fs["show_temp"];             // 是否显示模板图片

  offsets.push_back(cv::Point2i(offset_x_1, offset_y_1));
  offsets.push_back(cv::Point2i(offset_x_2, offset_y_2));
  offsets.push_back(cv::Point2i(offset_x_3, offset_y_3));
  offsets.push_back(cv::Point2i(offset_x_4, offset_y_4));
  offsets.push_back(cv::Point2i(offset_x_5, offset_y_5));
  offsets.push_back(cv::Point2i(offset_x_6, offset_y_6));
  offsets.push_back(cv::Point2i(offset_x_7, offset_y_7));
  offsets.push_back(cv::Point2i(offset_x_8, offset_y_8));
  offsets.push_back(cv::Point2i(offset_x_9, offset_y_9));
  offsets.push_back(cv::Point2i(offset_x_10, offset_y_10));
}

cv::Mat cylindrical_projection(cv::Mat img, int cylinder_method) {
  switch (cylinder_method) {
  case 1:
    img = cylinder1(img, f);
    break;
  case 2:
    img = cylinder2(img, f);
    break;
  case 3:
    img = cylinder3(img, f);
    break;
  case 4:
    img = cylinder4(img, f);
    break;
  case 5:
    img = cylinder5(img, f);
    break;
  case 6:
    img = cylinder6(img, f);
    break;
  case 7:
    img = cylinder7(img, f);
    break;
  case 8:
    img = cylinder8(img, f);
    break;
  case 9:
    img = cylinder9(img, f);
    break;
  case 10:
    img = cylinder10(img, f);
    break;
  case 11:
    img = cylinder11(img, f);
    break;
  case 12:
    img = cylinder12(img, f);
    break;
  case 13:
    img = cylinder13(img, f);
    break;
  default:
    break;
  }
  return img;
}

cv::Mat stitch_image(cv::Mat stitch, cv::Mat img1, cv::Mat img2, cv::Point2i a,
                     int stitch_method) {
  switch (stitch_method) {
  case 1:
    stitch = linearStitch(img1, img2, a);
    break;
  case 2:
    stitch = linearStitch_parallel_for_(img1, img2, a);
    break;
  case 3:
    stitch = linearStitch_openMP(img1, img2, a);
    break;
  case 4:
    stitch = linearStitch_parallel_for_mul(img1, img2, a);
    break;
  case 5:
    stitch = linearStitch_parallel_for_mul_not_cut(img1, img2, a);
    break;
  case 6:
    stitch = linearStitch_parallel_for_alpha_blending(img1, img2, a);
    break;
  default:
    break;
  }
  return stitch;
}

int main(int argc, char **argv) {
  // 读取参数
  get_param();

  if (flag_mul) {
    int image_num = argc - 1;
    std::cout << "一共有" << image_num << "张图片待拼接" << std::endl;
    std::vector<cv::Mat> images;
    for (int i = 1; i < argc; i++) {
      cv::Mat img = cv::imread(argv[i]);
      if (is_resize) {
        cv::resize(img, img, cv::Size(re_cols, re_rows), cv::INTER_LINEAR);
      }
      img = cylindrical_projection(img, cylinder_method);
      cv::imwrite("../result/cylinder_" + std::to_string(i) + ".jpg", img);
      images.push_back(img);
    }

    multi_image_stitch(images, image_num, is_cal_offset, offset_x, offset_y,
                       stitch_method);

  } else {
    cv::Mat img1 = cv::imread(argv[1]); // 左图
    cv::Mat img2 = cv::imread(argv[2]); // 右图
    if (flag_it) {
      cal_avg_time(img1, img2, is_resize, f, re_cols, re_rows, cylinder_method,
                   stitch_method, is_cal_offset, offset_x, offset_y, it_num);
    } else {
      show_stitch(img1, img2, is_resize, f, re_cols, re_rows, cylinder_method,
                  stitch_method, is_cal_offset, offset_x, offset_y);
    }
  }
  return 0;
}