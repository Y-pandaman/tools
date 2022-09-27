#pragma once

#include <fstream>
#include <iostream>
#include <ostream>
#include <queue>
#include <string>
#include <vector>

// opencv
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;

#include "ElapsedTime.h"
#include "GMSMatcher/gms_matcher.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

class MiscUtils {
public:
  static string type2str(int type);
  static string cvmat_info(const cv::Mat &mat);

  static std::vector<std::string> split(std::string const &original,
                                        char separator);

  //---------------------------- Conversions ---------------------------------//

  // given opencv keypoints and DMatch will produce M1, and M2 the co-ordinates
  static void dmatch_2_eigen(const std::vector<cv::KeyPoint> &kp1,
                             const std::vector<cv::KeyPoint> &kp2,
                             const std::vector<cv::DMatch> matches,
                             MatrixXd &M1, MatrixXd &M2,
                             bool make_homogeneous = true);

  //------------------------------- Plot Matchings on image pair
  //-------------------------//

  // nearly same as the above, but will color every co-ordinate with different
  // color color_map_direction : 0 ==> // horizontal-gradiant
  //                       1 ==>  // vertical-gradiant
  //                       2 ==> // manhattan-gradiant
  //                       3 ==> // image centered manhattan-gradiant
  static void plot_point_pair(const cv::Mat &imA, const MatrixXd &ptsA,
                              int idxA, const cv::Mat &imB,
                              const MatrixXd &ptsB, int idxB, cv::Mat &dst,
                              short color_map_direction,
                              const string &msg = string("N.A"));

  //------------------------------- Plot Matchings on image pair
  //-------------------------//

  //------------------------- Points and Lines on Images
  //--------------------------------//

  // Given two image-points draw line between them, extend both ways. Infinite
  // line-segments
  static void draw_fullLine(cv::Mat &img, cv::Point2f a, cv::Point2f b,
                            cv::Scalar color);

  // draw line on the image, given a line equation in homogeneous co-ordinates.
  // l = (a,b,c) for ax+by+c = 0
  static void draw_line(const Vector3d l, cv::Mat &im, cv::Scalar color);

  // mark point on the image, pt is in homogeneous co-ordinate.
  static void draw_point(const Vector3d pt, cv::Mat &im, cv::Scalar color);

  // mark point on image
  // static void draw_point( const Vector2d pt, cv::Mat& im, cv::Scalar color );

  // END ------------------------- Points and Lines on Images
  // --------------------------------//

  // [Input] : f a float between 0 and 1.
  // [Output]: cv::Scalar gives out a bgr color pallet.
  // Note: This is inefficient, don't use it too often. If you are going to do
  // lot of quering for colors use `class FalseColors`
  static cv::Scalar getFalseColor(float f);

private:
  static double Slope(int x0, int y0, int x1, int y1);
};
