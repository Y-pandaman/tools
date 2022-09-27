// Sample usage for class CameraGeometry.h/MonoGeometry and
// CameraGeometry.h/StereoGeometry These classes abstractout the Stereo
// Geometry, Undistort etc. They can be used with any of the Camodocal cameras
// ie. Mei, Kannala-brandt, pinhole (ofcourse).
//  The whole idea of the abstract Camera clases and Geometry class is to make
//  the codebase truely object oriented and the core geometry abstracted.
//  Hopefully all this will help to develop more higher level applications
//  faster.

// YONGYEN'S METHOD TO CORRECT THE STEREO-EXTRINSIC WHOLLY CONTAINED IN THIS
// FILE implements Yonggen Ling's method Y. Ling and S. Shen, "High-precision
// online markerless stereo extrinsic calibration," 2016 IEEE/RSJ International
// Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, pp.
// 1771-1778.

// minimize_{R,t} \sum_i || (f'_i)^T E f_i || , where R,t == right_T_left.
//           a) E is the Essential matrix E := [t]_x R
//           b) f and f' are point feature matches (f from left f from right) in
//           normalized image co-ordinates

#include <boost/format.hpp>
#include <iostream>
#include <string>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

// opencv
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "utils/CameraGeometry.h"
#include "utils/ElapsedTime.h"
#include "utils/MiscUtils.h"
#include "utils/PoseManipUtils.h"
#include "utils/RawFileIO.h"
#include "utils/StereoCalibOpt.h"
#include "utils/TermColor.h"

#include "utils/GMSMatcher/gms_matcher.h"

#include <assert.h>

#include <ceres/ceres.h>
using namespace ceres;

int g_picture_num = 11;
// float g_rotation_noise = 0;
// float g_tran_noise = 0;

// 提取外参
bool LoadExtrinsics(const std::string &filename, Eigen::Matrix4d &Trans) {
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    return false;
  }

  Vector4d q_xyzw = Vector4d(0, 0, 0, 1);
  Vector3d t_xyz = Vector3d::Zero();

  cv::FileNode node = fs["transform"];
  q_xyzw[0] = static_cast<double>(node["q_x"]);
  q_xyzw[1] = static_cast<double>(node["q_y"]);
  q_xyzw[2] = static_cast<double>(node["q_z"]);
  q_xyzw[3] = static_cast<double>(node["q_w"]);

  t_xyz[0] = static_cast<double>(node["t_x"]);
  t_xyz[1] = static_cast<double>(node["t_y"]);
  t_xyz[2] = static_cast<double>(node["t_z"]);

  Quaterniond q = Quaterniond(q_xyzw(3), q_xyzw(0), q_xyzw(1), q_xyzw(2));

  Trans = Eigen::Matrix4d::Identity();
  Trans.topLeftCorner<3, 3>() = q.toRotationMatrix();

  // 从原矩阵第（0，3）开始获取一个3行1列的子矩阵
  Trans.block<3, 1>(0, 3) = t_xyz;

  return true;
}

// 计算极线误差
void print_epipolar_error(const cv::Mat &imleft_undistorted,
                          const cv::Mat &imright_undistorted) {
  // 提取ORB特征
  std::vector<cv::KeyPoint> kp1, kp2;
  cv::Mat d1, d2; // 描述子

  cv::Ptr<cv::ORB> orb = cv::ORB::create(3000);
  orb->setFastThreshold(0);
  orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
  orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);

  // 计算汉明距离
  cv::BFMatcher matcher(cv::NORM_HAMMING, true); // TODO try FLANN matcher here.
  vector<cv::DMatch> matches_all;
  matcher.match(d1, d2, matches_all);

  // 过滤
  vector<DMatch> goodMatches;
  double minDis = 9999;
  //  找到最小距离
  for (size_t i = 0; i < matches_all.size(); i++) {
    if (matches_all[i].distance < minDis)
      minDis = matches_all[i].distance;
  }

  // 设置阈值
  minDis = std::max(2 * minDis, 30.);
  for (size_t i = 0; i < matches_all.size(); i++) {
    if (matches_all[i].distance < minDis)
      goodMatches.push_back(matches_all[i]);
  }

  // 获得初匹配的特征点
  std::vector<Point2f> pts1, pts2;
  for (size_t i = 0; i < goodMatches.size(); i++) {
    pts1.push_back(kp1[goodMatches[i].queryIdx].pt);
    pts2.push_back(kp2[goodMatches[i].trainIdx].pt);
  }

  Mat img_match;
  cv::drawMatches(imleft_undistorted, kp1, imright_undistorted, kp2,
                  goodMatches, img_match);
  cv::imshow("goodmatch", img_match);

  if (pts1.empty()) {
    cout << "goodMatches is empty\n";
    return;
  }

  double y_error_bef = 0;
  double error_after_ransac = 0;

  // 计算初匹配的极线误差，像素y
  for (size_t i = 0; i < pts1.size(); i++) {
    y_error_bef += std::fabs(pts1[i].y - pts2[i].y);
  }

  cout << "y_error_bef: " << y_error_bef / pts1.size() << endl;

  // -----------RANSAC 过滤----------
  std::vector<uchar> listpoints;
  Mat H = findHomography(pts1, pts2, listpoints, CV_RANSAC, 5.0);

  std::vector<DMatch> betterMatches;
  for (size_t i = 0; i < listpoints.size(); i++) {
    if ((int)listpoints[i]) {
      betterMatches.push_back(goodMatches[i]);
    }
  }
  //   goodMatches.swap(betterMatches);  // 释放空内存

  // 获得RANSAC后匹配的特征点
  std::vector<Point2f> bpts1, bpts2;
  for (size_t i = 0; i < betterMatches.size(); i++) {
    bpts1.push_back(kp1[betterMatches[i].queryIdx].pt);
    bpts2.push_back(kp2[betterMatches[i].trainIdx].pt);
  }

  // 过滤误差大的点
  for (size_t i = 0; i < bpts1.size(); i++) {
    if (std::fabs(bpts1[i].y - bpts2[i].y) > 1.2) {
      bpts1.erase(bpts1.begin() + i);
      bpts2.erase(bpts2.begin() + i);
      betterMatches.erase(betterMatches.begin() + i);
    }
  }

  // 计算误差
  for (size_t i = 0; i < bpts1.size(); i++) {
    error_after_ransac += std::fabs(bpts1[i].y - bpts2[i].y);
  }
  cout << "error_after_ransac:" << error_after_ransac / bpts1.size() << endl;
  // -----------RANSAC 过滤----------//

  Mat img_match1;
  cv::drawMatches(imleft_undistorted, kp1, imright_undistorted, kp2,
                  betterMatches, img_match1);
  cv::imshow("ransac_match", img_match1);

  // cv::waitKey(0);
  // cv::destroyAllWindows();
}

// GMS匹配
void point_feature_matches(const cv::Mat &imleft_undistorted,
                           const cv::Mat &imright_undistorted, MatrixXd &u,
                           MatrixXd &ud) {
  ElapsedTime timer;

  // Point feature and descriptors extract
  std::vector<cv::KeyPoint> kp1, kp2;
  cv::Mat d1, d2; //< descriptors

  cv::Ptr<cv::ORB> orb = cv::ORB::create(3000);
  orb->setFastThreshold(0);

  timer.tic();
  orb->detectAndCompute(imleft_undistorted, cv::Mat(), kp1, d1);
  orb->detectAndCompute(imright_undistorted, cv::Mat(), kp2, d2);
  cout << "2X detectAndCompute(ms) : " << timer.toc_milli() << endl;
  // std::cout << "d1 " << MiscUtils::cvmat_info( d1 ) << std::endl;
  // std::cout << "d2 " << MiscUtils::cvmat_info( d2 ) << std::endl;

  // plot
  // cv::Mat dst_left, dst_right;
  // MatrixXd e_kp1, e_kp2;
  // MiscUtils::keypoint_2_eigen( kp1, e_kp1 );
  // MiscUtils::keypoint_2_eigen( kp2, e_kp2 );
  // MiscUtils::plot_point_sets( imleft_undistorted, e_kp1, dst_left,
  // cv::Scalar(0,0,255), false ); MiscUtils::plot_point_sets(
  // imright_undistorted, e_kp2, dst_right, cv::Scalar(0,0,255), false );
  // cv::imshow( "dst_left", dst_left );
  // cv::imshow( "dst_right", dst_right );

  //
  // Point feature matching
  cv::BFMatcher matcher(cv::NORM_HAMMING, true); // TODO try FLANN matcher here.
  vector<cv::DMatch> matches_all, matches_gms;
  timer.tic();
  matcher.match(d1, d2, matches_all);
  std::cout << "BFMatcher : npts = " << matches_all.size() << std::endl;
  std::cout << "BFMatcher took (ms) : " << timer.toc_milli() << std::endl;

  // gms_matcher
  timer.tic();
  std::vector<bool> vbInliers;
  gms_matcher gms(kp1, imleft_undistorted.size(), kp2,
                  imright_undistorted.size(), matches_all);
  int num_inliers = gms.GetInlierMask(vbInliers, false, false);
  cout << "Got total gms matches " << num_inliers << " matches." << endl;
  cout << "GMSMatcher took (ms) " << timer.toc_milli() << std::endl;

  // collect matches
  for (size_t i = 0; i < vbInliers.size(); ++i) {
    if (vbInliers[i] == true) {
      matches_gms.push_back(matches_all[i]);
    }
  }
  // MatrixXd M1, M2;
  MiscUtils::dmatch_2_eigen(kp1, kp2, matches_gms, u, ud, true);
}

// ceres优化
void nudge_extrinsics_multi(std::vector<MatrixXd> f_left_vec,
                            std::vector<MatrixXd> f_right_vec,
                            const Matrix4d &right_T_left,
                            Matrix4d &optimized_right_T_left) {

  // Step-3 : Setup CERES problem
  // 3.1 : Initial Guess
  double T_cap_q[10], T_cap_t[10];
  PoseManipUtils::eigenmat_to_raw(right_T_left, T_cap_q, T_cap_t);
  double n_norm = std::sqrt(T_cap_t[0] * T_cap_t[0] + T_cap_t[1] * T_cap_t[1] +
                            T_cap_t[2] * T_cap_t[2]);
  T_cap_t[0] /= n_norm;
  T_cap_t[1] /= n_norm;
  T_cap_t[2] /= n_norm;

  cout << "CERES Inital Guess : "
       << PoseManipUtils::prettyprintMatrix4d(right_T_left) << endl;
  cout << "CERES Initial Guess n_norm: " << n_norm << endl;
  cout << "CERES Initial Guess T_cap_t: " << T_cap_t[0] << " " << T_cap_t[1]
       << " " << T_cap_t[2] << endl;

  // 3.2 : Error Terms
  ceres::Problem problem;
  // cout << "CERES #residues : " << f_left.cols() << endl;

  for (int i = 0; i < g_picture_num; i++) {
    MatrixXd f_left = f_left_vec[i];
    MatrixXd f_right = f_right_vec[i];
    for (int i = 0; i < f_left.cols(); i++) {
      int r = rand() % f_left.cols();
      CostFunction *cost_function =
          YonggenResidue::Create(f_left.col(r).head(3), f_right.col(r).head(3));
      // problem.AddResidualBlock( cost_function, NULL, T_cap_q, T_cap_t );
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.001),
                               T_cap_q, T_cap_t);
    }
  }

  // 3.3 : Local Parameterization
  ceres::LocalParameterization *quaternion_parameterization =
      new ceres::QuaternionParameterization;
  ceres::LocalParameterization *hv_parameterization =
      new UnitVectorParameterization();
  problem.SetParameterization(T_cap_q, quaternion_parameterization);
  problem.SetParameterization(T_cap_t, hv_parameterization);

  // Step-4 : Solve
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  // Step-5 : Retrive Solution
  Matrix4d T_cap;
  PoseManipUtils::raw_to_eigenmat(T_cap_q, T_cap_t, T_cap);
  cout << "CERES Solution : " << PoseManipUtils::prettyprintMatrix4d(T_cap)
       << endl;

  Vector3d tran = T_cap.col(3).topRows(3);
  // T_cap.col(3).topRows(3) *= n_norm;
  T_cap.col(3).topRows(3) = tran / tran.norm() * 0.1;

  optimized_right_T_left = T_cap;
  cout << "CERES Solution T_cap(after rescaling): "
       << PoseManipUtils::prettyprintMatrix4d(T_cap) << endl;
}

// Stereo
int stereo_demo() {
  IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");
  ElapsedTime timer;
  cv::FileStorage fs("/home/bdca/workspace/catkin_ws/src/"
                     "markerless_stereo_calib/config/config.yaml",
                     cv::FileStorage::READ);
  string left_intrinsic, right_intrinsic, extrinsic, image_base, image_path;
  int frame_id, noise_flag;
  float rotation_noise, tran_noise;

  fs["left_intrinsic"] >> left_intrinsic;
  fs["right_intrinsic"] >> right_intrinsic;
  fs["extrinsic"] >> extrinsic;
  fs["image_base"] >> image_base;
  fs["frame_id"] >> frame_id;
  fs["image_path"] >> image_path;
  fs["rotation_noise"] >> rotation_noise;
  fs["tran_noise"] >> tran_noise;
  fs["noise_flag"] >> noise_flag;

  std::string left_image_path =
      image_base + "/cam0/" + std::to_string(frame_id) + ".png";
  std::string right_image_path =
      image_base + "/cam1/" + std::to_string(frame_id) + ".png";

  // Abstract Camera
  camodocal::CameraPtr left_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          left_intrinsic);
  camodocal::CameraPtr right_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          right_intrinsic);

  // Extrinsics
  Matrix4d right_T_left;
  LoadExtrinsics(extrinsic, right_T_left);

  cout << left_camera->parametersToString() << endl;
  cout << right_camera->parametersToString() << endl;

  cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d(right_T_left)
       << endl;
  cout << "right_T_left=\n" << right_T_left.format(numpyFmt) << endl;

  if (noise_flag) {
    // add noise
    Matrix4d delta;
    PoseManipUtils::rawyprt_to_eigenmat(
        Vector3d(rotation_noise, rotation_noise, rotation_noise),
        Vector3d(tran_noise, tran_noise, tran_noise), delta);
    right_T_left = (delta * right_T_left).eval();
    cout << "right_T_left(after applying delta): "
         << PoseManipUtils::prettyprintMatrix4d(right_T_left) << endl;
  }

  std::shared_ptr<StereoGeometry> stereogeom;
  stereogeom =
      std::make_shared<StereoGeometry>(left_camera, right_camera, right_T_left);

  Eigen::Matrix3d new_cam_K;
  GeometryUtils::getK(left_camera, new_cam_K);
  stereogeom->set_K(new_cam_K);

  std::vector<cv::Mat> imleft_raw, imright_raw;
  for (size_t i = 1; i <= g_picture_num; i++) {
    boost::format fmt(image_path + "/%s/%d.%s");

    // Raw Image - Image from camera
    imleft_raw.push_back(cv::imread((fmt % "cam0" % i % "png").str(), 0));
    imright_raw.push_back(cv::imread((fmt % "cam1" % i % "png").str(), 0));
  }

  std::vector<MatrixXd> f_left_vec;
  std::vector<MatrixXd> f_right_vec;

  timer.tic();
  for (size_t i = 0; i < g_picture_num; i++) {
    std::cout << "-------第" << i + 1 << "张图片----------" << std::endl;
    // Undistort Only
    cv::Mat imleft_undistorted, imright_undistorted;
    stereogeom->do_image_undistortion(imleft_raw[i], imright_raw[i],
                                      imleft_undistorted, imright_undistorted);
    cout << " ======= before opt extrinsics ======= \n";
    {
      cv::Mat imleft_srectified, imright_srectified;
      stereogeom->do_stereo_rectification_of_undistorted_images(
          imleft_undistorted, imright_undistorted, imleft_srectified,
          imright_srectified);
      // print_epipolar_error(imleft_srectified, imright_srectified);
    }

    // Step-1 : Match point features from `imleft_undistorted,
    // imright_undistorted`
    MatrixXd ul, ur; //< 3xN, ie. image co-ordinates represented in homogeneous
                     // cord system.
    point_feature_matches(imleft_undistorted, imright_undistorted, ul, ur);
    cv::Mat dst;
    MiscUtils::plot_point_pair(imleft_undistorted, ul, -1, imright_undistorted,
                               ur, -1, dst, 0); 

    // Step-2 : In normalized co-ordinates : inv(K_new) * f' and inv(K_inv) * f
    MatrixXd fl = stereogeom->get_K().inverse() * ul;
    MatrixXd fr = stereogeom->get_K().inverse() * ur;
    f_left_vec.push_back(fl);
    f_right_vec.push_back(fr);
  }
  cout << "提取特征点耗时 (ms) " << timer.toc_milli() << std::endl;

  // Stereo calibration
  {
    timer.tic();
    Matrix4d optimized_right_T_left;
    nudge_extrinsics_multi(f_left_vec, f_right_vec, right_T_left,
                           optimized_right_T_left);
    cout << "optimized_right_T_left: "
         << PoseManipUtils::prettyprintMatrix4d(optimized_right_T_left) << endl;
    stereogeom->set_stereoextrinsic(optimized_right_T_left);
    cout << "优化耗时 (ms) " << timer.toc_milli() << std::endl;
  }

  // will get 3d points, stereo-rectified image, and disparity false colormap
  MatrixXd _3dpts; // 4xN
  cv::Mat imleft_srectified, imright_srectified;
  cv::Mat disparity_for_visualization;
  cv::Mat imleft = cv::imread(left_image_path, 0);
  cv::Mat imright = cv::imread(right_image_path, 0);

  stereogeom->get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(
      imleft, imright, imleft_srectified, imright_srectified, _3dpts,
      disparity_for_visualization);

  cout << " ======= after opt extrinsics ======= \n";
  print_epipolar_error(imleft_srectified, imright_srectified);

  cv::imshow("disparity_for_visualization", disparity_for_visualization);

  // Draw Epipolar lines on stereo rectified images
  cv::Mat dst_imleft_srectified = imleft_srectified.clone();
  cv::Mat dst_imright_srectified = imright_srectified.clone();
  stereogeom->draw_srectified_epipolarlines(dst_imleft_srectified,
                                            dst_imright_srectified);

  Mat canvas;
  int w = dst_imleft_srectified.cols;
  int h = dst_imleft_srectified.rows;
  canvas.create(h, w * 2, CV_8UC3);
  dst_imleft_srectified.copyTo(canvas(cv::Rect(0, 0, w, h)));
  dst_imright_srectified.copyTo(canvas(cv::Rect(w, 0, w, h)));
  cv::imshow("img_srectified", canvas);

  cv::waitKey(0);
}

int main() { stereo_demo(); }
