#ifndef GPL_H
#define GPL_H

#include <algorithm>
#include <cmath>
#include <opencv2/core/core.hpp>

namespace camodocal {

template <class T> const T clamp(const T &v, const T &a, const T &b) {
  return std::min(b, std::max(a, v));
}

double hypot3(double x, double y, double z);
float hypot3f(float x, float y, float z);

template <class T> const T normalizeTheta(const T &theta) {
  T normTheta = theta;

  while (normTheta < -M_PI) {
    normTheta += 2.0 * M_PI;
  }
  while (normTheta > M_PI) {
    normTheta -= 2.0 * M_PI;
  }

  return normTheta;
}

template <class T> const T square(const T &x) { return x * x; }

template <class T> const T cube(const T &x) { return x * x * x; }

template <class T> const T random(const T &a, const T &b) {
  return static_cast<double>(rand()) / RAND_MAX * (b - a) + a;
}

template <class T> const T randomNormal(const T &sigma) {
  T x1, x2, w;

  do {
    x1 = 2.0 * random(0.0, 1.0) - 1.0;
    x2 = 2.0 * random(0.0, 1.0) - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w >= 1.0 || w == 0.0);

  w = sqrt((-2.0 * log(w)) / w);

  return x1 * w * sigma;
}

unsigned long long timeInMicroseconds(void);

double timeInSeconds(void);

void fitCircle(const std::vector<cv::Point2d> &points, double &centerX,
               double &centerY, double &radius);

std::vector<cv::Point2d> intersectCircles(double x1, double y1, double r1,
                                          double x2, double y2, double r2);

long int timestampDiff(uint64_t t1, uint64_t t2);

} // namespace camodocal

#endif
