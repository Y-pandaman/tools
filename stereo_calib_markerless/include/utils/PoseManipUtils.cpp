#include "PoseManipUtils.h"

void PoseManipUtils::raw_to_eigenmat( const double * quat, const double * t, Matrix4d& dstT )
{
  Quaterniond q = Quaterniond( quat[0], quat[1], quat[2], quat[3] );

  dstT = Matrix4d::Zero();
  dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

  dstT(0,3) = t[0];
  dstT(1,3) = t[1];
  dstT(2,3) = t[2];
  dstT(3,3) = 1.0;
}

void PoseManipUtils::eigenmat_to_raw( const Matrix4d& T, double * quat, double * t)
{
  assert( T(3,3) == 1 );
  Quaterniond q( T.topLeftCorner<3,3>() );
  quat[0] = q.w();
  quat[1] = q.x();
  quat[2] = q.y();
  quat[3] = q.z();
  t[0] = T(0,3);
  t[1] = T(1,3);
  t[2] = T(2,3);
}

void PoseManipUtils::raw_xyzw_to_eigenmat( const Vector4d& quat, const Vector3d& t, Matrix4d& dstT )
{
  Quaterniond q = Quaterniond( quat(3), quat(0), quat(1), quat(2) );

  dstT = Matrix4d::Zero();
  dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

  dstT(0,3) = t(0);
  dstT(1,3) = t(1);
  dstT(2,3) = t(2);
  dstT(3,3) = 1.0;
}

// input ypr must be in degrees.
void PoseManipUtils::rawyprt_to_eigenmat( const Vector3d& eigen_ypr_degrees, const Vector3d& t, Matrix4d& dstT )
{
  dstT = Matrix4d::Identity();
  dstT.topLeftCorner<3,3>() = ypr2R( eigen_ypr_degrees );
  dstT(0,3) = t(0);
  dstT(1,3) = t(1);
  dstT(2,3) = t(2);
}

Vector3d PoseManipUtils::R2ypr( const Matrix3d& R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}


Matrix3d PoseManipUtils::ypr2R( const Vector3d& ypr)
{
  double y = ypr(0) / 180.0 * M_PI;
  double p = ypr(1) / 180.0 * M_PI;
  double r = ypr(2) / 180.0 * M_PI;

  // Eigen::Matrix<double, 3, 3> Rz;
  Matrix3d Rz;
  Rz << cos(y), -sin(y), 0,
      sin(y), cos(y), 0,
      0, 0, 1;

  // Eigen::Matrix<double, 3, 3> Ry;
  Matrix3d Ry;
  Ry << cos(p), 0., sin(p),
      0., 1., 0.,
      -sin(p), 0., cos(p);

  // Eigen::Matrix<double, 3, 3> Rx;
  Matrix3d Rx;
  Rx << 1., 0., 0.,
      0., cos(r), -sin(r),
      0., sin(r), cos(r);

  return Rz * Ry * Rx;
}

string PoseManipUtils::prettyprintMatrix4d( const Matrix4d& M )
{
   Vector3d ypr;
   ypr = R2ypr(  M.topLeftCorner<3,3>()  );

  char __tmp[200];
  snprintf( __tmp, 200, ":YPR(deg)=(%4.3f,%4.3f,%4.3f)  :TxTyTz=(%4.3f,%4.3f,%4.3f)",  ypr(0), ypr(1), ypr(2), M(0,3), M(1,3), M(2,3) );
  string return_string = string( __tmp );
  return return_string;
}

void PoseManipUtils::vec_to_cross_matrix( const Vector3d& t, Matrix3d& A_x )
{
    // Tx = Matrix3d::Zero();
    A_x << 0, -t(2) , t(1) ,
          t(2), 0,  -t(0),
          -t(1), t(0), 0;
}
