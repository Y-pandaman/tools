#pragma once

#include <ceres/ceres.h>

using namespace ceres;
using namespace Eigen;

// Orthonormalization is needed because the optimization variable translation is
// unit normalized and to preserve this property we need to define the '+'
// operation for it. Note that it can only more in tangent space of the sphere.
// So graph schmit-orthonormalization gives out a basis vector at this point.
// Ofcourse this is not a unique solution. See Fig. 3 in yonggen's iros2016
// paper.
class UnitVectorParameterization : public ceres::LocalParameterization {
public:
  UnitVectorParameterization() {}
  virtual ~UnitVectorParameterization() {}

  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const {
    // Compute T (tangent space)
    // tangent space is a function of x
    double m[3], n[3];
    bool statis = gram_schmidt_orthonormalization(x, m, n);
    if (!statis)
      return false;

    // x_new := x + T*[ delta[0]; delta[1] ]
    x_plus_delta[0] = x[0] + m[0] * delta[0] + n[0] * delta[1];
    x_plus_delta[1] = x[1] + m[1] * delta[0] + n[1] * delta[1];
    x_plus_delta[2] = x[2] + m[2] * delta[0] + n[2] * delta[1];
    return true;
  }

  virtual bool ComputeJacobian(const double *x, double *jacobian) const {
    // Compute T (tangent space)
    // tangent space is a function of x
    double m[3], n[3];
    bool statis = gram_schmidt_orthonormalization(x, m, n);
    if (!statis)
      return false;

    // \del(x') / \del(a) = [  T_{0,0}; T_{1,0}  ]

    // \del(x') / \del(b) = [  T_{0,1}; T_{1,1}  ]

    // jacobian = [ concate above 2 ]
    jacobian[0] = m[0];
    jacobian[1] = n[0];
    jacobian[2] = m[1];
    jacobian[3] = n[1];
    jacobian[4] = m[2];
    jacobian[5] = n[2];
    return true;
  }
  virtual int GlobalSize() const { return 3; } // TODO : Generalize.
  virtual int LocalSize() const { return 2; }

  // This function return 2 vectors m, n each of dimension equal to x (ie. 3)
  // such that x,m,n are orthogonal to each other.
  const bool gram_schmidt_orthonormalization(const double *_x, double *_m,
                                             double *_n, int len = 3) const {
    VectorXd xp = VectorXd::Zero(len);
    for (int i = 0; i < len; i++)
      xp(i) = _x[i];

    // xp = xp / xp.norm();

    // equation of tangent-plane : x_p * x  + y_p * y + z_p * z  = 1,
    // where (xp,yp,zp) is a known point on the sphere,. point passing through
    // (x_p,y_p,z_p) and plane normal vector direction as (x_p, y_p, z_p).

    // assert( abs(xp.norm()-1.) < 1e-7 );
    // if( abs(xp(2)) < 1E-5 )
    // return false;

    VectorXd m = VectorXd::Zero(len); //< a point on tangent plane.
    m << 10., -7,
        1.0 - (xp(0) * 10. + xp(1) * (-7.)) /
                  xp(2); //< assuming xp(2) is non-zero, TODO ideally should
                         // divide by the maximum of {xp(0), xp(1), xp(2) }, the
                         // points will change accordingly

    VectorXd n = VectorXd::Zero(len); //< another point on tangent place.
    n << -12., 5,
        1.0 - (xp(0) * (-12.) + xp(1) * (5.)) /
                  xp(2); //< assuming xp(2) is non-zero

    m = m - xp;
    m /= m.norm(); //< unit vector in the direction of m

    n = n - xp;
    n /= n.norm(); //< unit vector in the direction of n

    m = m - proj(xp, m); // projection of xp in the direction of m
    m = m / m.norm();
    n = n - proj(xp, n) - proj(m, n);
    n = n / n.norm();

    // m, n are the 2 basis vectors.
    // cout << "m" << m << endl;
    // cout << "n" << n << endl;
    for (int i = 0; i < len; i++) {
      _m[i] = m(i);
      _n[i] = n(i);
    }
    return true;
  }

  VectorXd proj(const VectorXd &u, const VectorXd &v) const {
    return u * (u.dot(v) / u.dot(u));
  }
};

class YonggenResidue {
public:
  YonggenResidue(const Vector3d &Xi, const Vector3d &Xid) : Xi(Xi), Xid(Xid) {
    // this->Xi = Xi;
    // this->Xid = Xid;
  }

  template <typename T>
  bool operator()(const T *const q, const T *const t, T *residual) const {
    // Optimization variables
    Quaternion<T> eigen_q(q[0], q[1], q[2], q[3]);

    Eigen::Matrix<T, 3, 3> eigen_tx;
    eigen_tx << T(0.0), -t[2], t[1], t[2], T(0.0), -t[0], -t[1], t[0], T(0.0);

    Eigen::Matrix<T, 3, 3> essential_matrix;
    essential_matrix = eigen_tx * eigen_q.toRotationMatrix();

    // Known Constant
    Eigen::Matrix<T, 3, 1> eigen_Xi;
    Eigen::Matrix<T, 3, 1> eigen_Xid;
    eigen_Xi << T(Xi(0)), T(Xi(1)), T(Xi(2));
    eigen_Xid << T(Xid(0)), T(Xid(1)), T(Xid(2));

    // Error term

    Eigen::Matrix<T, 1, 1> e;
    // e = eigen_Xi - (  eigen_q.toRotationMatrix() * eigen_Xid + eigen_t );
    e = eigen_Xid.transpose() * essential_matrix * eigen_Xi;
    // e = eigen_Xi.transpose() * (essential_matrix * eigen_Xid);

    residual[0] = e(0);
    // residual[1] = e(1);
    // residual[2] = e(2);

    return true;
  }

  static ceres::CostFunction *Create(const Vector3d &_Xi,
                                     const Vector3d &_Xid) {
    return (new ceres::AutoDiffCostFunction<YonggenResidue, 1, 4, 3>(
        new YonggenResidue(_Xi, _Xid)));
  }

private:
  Vector3d Xi, Xid;
};
