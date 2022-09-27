#include "MiscUtils.h"

string MiscUtils::type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
  case CV_8U:
    r = "8U";
    break;
  case CV_8S:
    r = "8S";
    break;
  case CV_16U:
    r = "16U";
    break;
  case CV_16S:
    r = "16S";
    break;
  case CV_32S:
    r = "32S";
    break;
  case CV_32F:
    r = "32F";
    break;
  case CV_64F:
    r = "64F";
    break;
  default:
    r = "User";
    break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

string MiscUtils::cvmat_info(const cv::Mat &mat) {
  std::stringstream buffer;
  buffer << "shape=" << mat.rows << "," << mat.cols << "," << mat.channels();
  buffer << "\t"
         << "dtype=" << MiscUtils::type2str(mat.type());
  return buffer.str();
}

std::vector<std::string> MiscUtils::split(std::string const &original,
                                          char separator) {
  std::vector<std::string> results;
  std::string::const_iterator start = original.begin();
  std::string::const_iterator end = original.end();
  std::string::const_iterator next = std::find(start, end, separator);
  while (next != end) {
    results.push_back(std::string(start, next));
    start = next + 1;
    next = std::find(start, end, separator);
  }
  results.push_back(std::string(start, next));
  return results;
}

void MiscUtils::dmatch_2_eigen(const std::vector<cv::KeyPoint> &kp1,
                               const std::vector<cv::KeyPoint> &kp2,
                               const std::vector<cv::DMatch> matches,
                               MatrixXd &M1, MatrixXd &M2,
                               bool make_homogeneous) {
  assert(matches.size() > 0 &&
         "MiscUtils::dmatch_2_eigen DMatch cannot be empty");
  assert(kp1.size() > 0 && kp2.size() > 0 &&
         "MiscUtils::dmatch_2_eigen keypoints cannot be empty");

  M1 = MatrixXd::Constant((make_homogeneous ? 3 : 2), matches.size(), 1.0);
  M2 = MatrixXd::Constant((make_homogeneous ? 3 : 2), matches.size(), 1.0);
  for (int i = 0; i < matches.size(); i++) {
    int queryIdx = matches[i].queryIdx; // kp1
    int trainIdx = matches[i].trainIdx; // kp2
    assert(queryIdx >= 0 && queryIdx < kp1.size());
    assert(trainIdx >= 0 && trainIdx < kp2.size());
    M1(0, i) = kp1[queryIdx].pt.x;
    M1(1, i) = kp1[queryIdx].pt.y;

    M2(0, i) = kp2[trainIdx].pt.x;
    M2(1, i) = kp2[trainIdx].pt.y;
  }
}

void MiscUtils::plot_point_pair(const cv::Mat &imA, const MatrixXd &ptsA,
                                int idxA, const cv::Mat &imB,
                                const MatrixXd &ptsB, int idxB, cv::Mat &dst,
                                short color_map_direction, const string &msg) {
  // ptsA : ptsB : 2xN or 3xN
  assert(color_map_direction >= 0 && color_map_direction <= 3);
  assert(imA.rows == imB.rows && imA.rows > 0);
  assert(imA.cols == imB.cols && imA.cols > 0);
  assert(ptsA.cols() == ptsB.cols() && ptsA.cols() > 0);
  // assert( mask.size() == ptsA.cols() );

  // make colormap
  cv::Mat colormap_gray = cv::Mat::zeros(1, 256, CV_8UC1);
  for (int i = 0; i < 256; i++)
    colormap_gray.at<uchar>(0, i) = i;
  cv::Mat colormap_color;
  cv::applyColorMap(colormap_gray, colormap_color, cv::COLORMAP_HSV);

  cv::Mat outImg_;
  cv::hconcat(imA, imB, outImg_);

  cv::Mat outImg;
  if (outImg_.channels() == 3)
    outImg = outImg_;
  else
    cv::cvtColor(outImg_, outImg, CV_GRAY2BGR);

  cv::RNG rng;

  // loop over all points
  int count = 0;
  for (int kl = 0; kl < ptsA.cols(); kl++) {
    // if( mask(kl) == 0 )
    //   continue;

    count++;
    cv::Point2d A(ptsA(0, kl), ptsA(1, kl));
    cv::Point2d B(ptsB(0, kl), ptsB(1, kl));

    int coloridx;
    if (color_map_direction == 0)
      coloridx = (int)(ptsA(0, kl) / imA.cols * 256.); // horizontal-gradiant
    if (color_map_direction == 1)
      coloridx = (int)(ptsA(1, kl) / imA.rows * 256.); // vertical-gradiant
    if (color_map_direction == 2)
      coloridx = (int)((ptsA(0, kl) + ptsA(1, kl)) / (imA.rows + imA.cols) *
                       256.); // manhattan-gradiant
    if (color_map_direction == 3)
      coloridx =
          (int)(abs(ptsA(0, kl) - imA.rows / 2. + ptsA(1, kl) - imA.cols / 2.) /
                (imA.rows / 2. + imA.cols / 2.) *
                256.); // image centered manhattan-gradiant
    if (coloridx < 0 || coloridx > 255)
      coloridx = 0;
    cv::Vec3b f = colormap_color.at<cv::Vec3b>(0, (int)coloridx);
    cv::Scalar color_marker = cv::Scalar(f[0], f[1], f[2]);

    color_marker = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                              rng.uniform(0, 255));

    cv::circle(outImg, A, 2, color_marker, -1);
    cv::circle(outImg, B + cv::Point2d(imA.cols, 0), 2, color_marker, -1);

    /*
    cv::line( outImg,  A, B+cv::Point2d(imA.cols,0), color_line );

    if( annotate_pts )
    {
      cv::putText( outImg, to_string(kl), A, cv::FONT_HERSHEY_SIMPLEX, 0.3,
    color_marker, 1 ); cv::putText( outImg, to_string(kl),
    B+cv::Point2d(imA.cols,0), cv::FONT_HERSHEY_SIMPLEX, 0.3, color_marker, 1 );
    }
    */
  }

  cv::Mat status = cv::Mat(150, outImg.cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::putText(status, to_string(idxA).c_str(), cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(status, to_string(idxB).c_str(), cv::Point(imA.cols + 10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  cv::putText(status, "marked # pts: " + to_string(count), cv::Point(10, 60),
              cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 1.5);

  // put msg in status image
  if (msg.length() > 0) { // ':' separated. Each will go in new line
    std::vector<std::string> msg_tokens = split(msg, ';');
    for (int h = 0; h < msg_tokens.size(); h++)
      cv::putText(status, msg_tokens[h].c_str(), cv::Point(10, 80 + 20 * h),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255),
                  1.5);
  }

  cv::vconcat(outImg, status, dst);
}

double MiscUtils::Slope(int x0, int y0, int x1, int y1) {
  return (double)(y1 - y0) / (x1 - x0);
}

void MiscUtils::draw_fullLine(cv::Mat &img, cv::Point2f a, cv::Point2f b,
                              cv::Scalar color) {
  double slope = MiscUtils::Slope(a.x, a.y, b.x, b.y);

  cv::Point2f p(0, 0), q(img.cols, img.rows);

  p.y = -(a.x - p.x) * slope + a.y;
  q.y = -(b.x - q.x) * slope + b.y;

  cv::line(img, p, q, color, 1, 8, 0);
}

// draw line on the image, given a line equation in homogeneous co-ordinates. l
// = (a,b,c) for ax+by+c = 0
void MiscUtils::draw_line(const Vector3d l, cv::Mat &im, cv::Scalar color) {
  // C++: void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int
  // thickness=1, int lineType=8, int shift=0)
  if (l(0) == 0) {
    // plot y = -c/b
    cv::Point2f a(0.0, -l(2) / l(1));
    cv::Point2f a_(10.0, -l(2) / l(1));
    MiscUtils::draw_fullLine(im, a, a_, color);
    return;
  }

  if (l(1) == 0) {
    // plot x = -c/a
    cv::Point2f b(-l(2) / l(0), 0.0);
    cv::Point2f b_(-l(2) / l(0), 10.0);
    MiscUtils::draw_fullLine(im, b, b_, color);
    return;
  }

  cv::Point2f a(0.0, -l(2) / l(1));
  cv::Point2f b(-l(2) / l(0), 0.0);
  // cout << a << "<--->" << b << endl;
  // cv::line( im, a, b, cv::Scalar(255,255,255) );
  MiscUtils::draw_fullLine(im, a, b, color);
}

// mark point on the image, pt is in homogeneous co-ordinate.
void MiscUtils::draw_point(const Vector3d pt, cv::Mat &im, cv::Scalar color) {
  // C++: void circle(Mat& img, Point center, int radius, const Scalar& color,
  // int thickness=1, int lineType=8, int shift=0)
  cv::circle(im, cv::Point2f(pt(0) / pt(2), pt(1) / pt(2)), 2, color, -1);
}
