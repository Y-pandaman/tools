#pragma once
//
// This provides utilities for pose manipulation.
//
// Author : Manohar Kuse <mpkuse@connect.ust.hk>
//

#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// uncomment this to compile without ros dependency for this file
#define __PoseManipUtils__with_ROS 1

#ifdef __PoseManipUtils__with_ROS
#include <geometry_msgs/Pose.h>
#endif

using namespace std;
using namespace Eigen;

class PoseManipUtils {
public:
  static void raw_to_eigenmat(const double *quat, const double *t,
                              Matrix4d &dstT);
  static void eigenmat_to_raw(const Matrix4d &T, double *quat, double *t);

  static void rawyprt_to_eigenmat(const Vector3d &eigen_ypr_degrees,
                                  const Vector3d &t, Matrix4d &dstT);

  static Vector3d R2ypr(const Matrix3d &R);
  static Matrix3d ypr2R(const Vector3d &ypr); // input ypr must be in degrees.

  static string prettyprintMatrix4d(const Matrix4d &M);

  static void raw_xyzw_to_eigenmat(const Vector4d &quat, const Vector3d &t,
                                   Matrix4d &dstT);

  // Given a point convert it to cross-product matrix. A_x = [ [ 0, -c, -b ],
  // [c,0,-a], [-b,-a,0] ]
  static void vec_to_cross_matrix(const Vector3d &a, Matrix3d &A_x);
};
