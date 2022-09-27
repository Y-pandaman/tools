#include "camodocal/gpl/gpl.h"

#include <set>
#ifdef _WIN32
#include <winsock.h>
#else
#include <time.h>
#endif

const double WGS84_A = 6378137.0;
const double WGS84_ECCSQ = 0.00669437999013;

// Windows lacks fminf
#ifndef fminf
#define fminf(x, y) (((x) < (y)) ? (x) : (y))
#endif

namespace camodocal {

double hypot3(double x, double y, double z) {
  return sqrt(square(x) + square(y) + square(z));
}

float hypot3f(float x, float y, float z) {
  return sqrtf(square(x) + square(y) + square(z));
}

#ifdef _WIN32
#include <sys/timeb.h>
#include <sys/types.h>
#include <winsock.h>
LARGE_INTEGER
getFILETIMEoffset() {
  SYSTEMTIME s;
  FILETIME f;
  LARGE_INTEGER t;

  s.wYear = 1970;
  s.wMonth = 1;
  s.wDay = 1;
  s.wHour = 0;
  s.wMinute = 0;
  s.wSecond = 0;
  s.wMilliseconds = 0;
  SystemTimeToFileTime(&s, &f);
  t.QuadPart = f.dwHighDateTime;
  t.QuadPart <<= 32;
  t.QuadPart |= f.dwLowDateTime;
  return (t);
}

int clock_gettime(int X, struct timespec *tp) {
  LARGE_INTEGER t;
  FILETIME f;
  double microseconds;
  static LARGE_INTEGER offset;
  static double frequencyToMicroseconds;
  static int initialized = 0;
  static BOOL usePerformanceCounter = 0;

  if (!initialized) {
    LARGE_INTEGER performanceFrequency;
    initialized = 1;
    usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
    if (usePerformanceCounter) {
      QueryPerformanceCounter(&offset);
      frequencyToMicroseconds =
          (double)performanceFrequency.QuadPart / 1000000.;
    } else {
      offset = getFILETIMEoffset();
      frequencyToMicroseconds = 10.;
    }
  }
  if (usePerformanceCounter)
    QueryPerformanceCounter(&t);
  else {
    GetSystemTimeAsFileTime(&f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
  }

  t.QuadPart -= offset.QuadPart;
  microseconds = (double)t.QuadPart / frequencyToMicroseconds;
  t.QuadPart = microseconds;
  tp->tv_sec = t.QuadPart / 1000000;
  tp->tv_nsec = (t.QuadPart % 1000000) * 1000;
  return (0);
}
#endif

void fitCircle(const std::vector<cv::Point2d> &points, double &centerX,
               double &centerY, double &radius) {
  // D. Umbach, and K. Jones, A Few Methods for Fitting Circles to Data,
  // IEEE Transactions on Instrumentation and Measurement, 2000
  // We use the modified least squares method.
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_yy = 0.0;
  double sum_xxx = 0.0;
  double sum_xxy = 0.0;
  double sum_xyy = 0.0;
  double sum_yyy = 0.0;

  int n = points.size();
  for (int i = 0; i < n; ++i) {
    double x = points.at(i).x;
    double y = points.at(i).y;

    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
    sum_yy += y * y;
    sum_xxx += x * x * x;
    sum_xxy += x * x * y;
    sum_xyy += x * y * y;
    sum_yyy += y * y * y;
  }

  double A = n * sum_xx - square(sum_x);
  double B = n * sum_xy - sum_x * sum_y;
  double C = n * sum_yy - square(sum_y);
  double D =
      0.5 * (n * sum_xyy - sum_x * sum_yy + n * sum_xxx - sum_x * sum_xx);
  double E =
      0.5 * (n * sum_xxy - sum_y * sum_xx + n * sum_yyy - sum_y * sum_yy);

  centerX = (D * C - B * E) / (A * C - square(B));
  centerY = (A * E - B * D) / (A * C - square(B));

  double sum_r = 0.0;
  for (int i = 0; i < n; ++i) {
    double x = points.at(i).x;
    double y = points.at(i).y;

    sum_r += hypot(x - centerX, y - centerY);
  }

  radius = sum_r / n;
}

std::vector<cv::Point2d> intersectCircles(double x1, double y1, double r1,
                                          double x2, double y2, double r2) {
  std::vector<cv::Point2d> ipts;

  double d = hypot(x1 - x2, y1 - y2);
  if (d > r1 + r2) {
    // circles are separate
    return ipts;
  }
  if (d < fabs(r1 - r2)) {
    // one circle is contained within the other
    return ipts;
  }

  double a = (square(r1) - square(r2) + square(d)) / (2.0 * d);
  double h = sqrt(square(r1) - square(a));

  double x3 = x1 + a * (x2 - x1) / d;
  double y3 = y1 + a * (y2 - y1) / d;

  if (h < 1e-10) {
    // two circles touch at one point
    ipts.push_back(cv::Point2d(x3, y3));
    return ipts;
  }

  ipts.push_back(cv::Point2d(x3 + h * (y2 - y1) / d, y3 - h * (x2 - x1) / d));
  ipts.push_back(cv::Point2d(x3 - h * (y2 - y1) / d, y3 + h * (x2 - x1) / d));
  return ipts;
}
} // namespace camodocal
