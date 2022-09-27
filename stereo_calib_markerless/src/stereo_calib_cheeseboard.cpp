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
#include "utils/StereoCalibOpt.h"
#include "utils/TermColor.h"

#include "utils/GMSMatcher/gms_matcher.h"

#include <assert.h>

#include <ceres/ceres.h>
using namespace ceres;

float g_noise_r = 1; // 旋转扰动值
float g_noise_t = 0; // 平移扰动值

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
  int corner_row = 7;                                  // 棋盘格行
  int corner_col = 6;                                  // 棋盘格列
  CvSize patter_size = cvSize(corner_row, corner_col); // 每一行每一列的内角数
  int corner_num = corner_col * corner_row;            // 角点个数
  vector<Point2f> corners_left, corners_right;         // 存储角点坐标
  Mat left_img, right_img;
  if (imleft_undistorted.channels() > 1) {
    cvtColor(imleft_undistorted, left_img, CV_BGR2GRAY);
    cvtColor(imright_undistorted, right_img, CV_BGR2GRAY);
  } else {
    imleft_undistorted.copyTo(left_img);
    imright_undistorted.copyTo(right_img);
  }

  // 找像素角点
  bool found_corner_left =
      cv::findChessboardCorners(left_img, patter_size, corners_left, 0);
  bool found_corner_right =
      cv::findChessboardCorners(right_img, patter_size, corners_right, 0);

  cv::RNG rng;
  //  显示结果
  {
    Mat before_left_img, before_right_img;
    if (left_img.channels() > 1) {
      left_img.copyTo(before_left_img);
      right_img.copyTo(before_right_img);
    } else {
      cvtColor(left_img, before_left_img, CV_GRAY2RGB); //伪彩色图
      cvtColor(right_img, before_right_img, CV_GRAY2RGB);
    }

    for (size_t i = 0; i < corners_left.size(); i++) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                    rng.uniform(0, 255));
      cv::circle(before_left_img, corners_left[i], 5, color, 0);
      cv::circle(before_right_img, corners_right[i], 5, color, 0);
    }

    Mat canvas;
    int w = before_left_img.cols;
    int h = before_left_img.rows;
    canvas.create(h, w * 2, CV_8UC3);
    before_left_img.copyTo(canvas(cv::Rect(0, 0, w, h)));
    before_right_img.copyTo(canvas(cv::Rect(w, 0, w, h)));

    cv::imshow("matched_for_epipolar_cheeseboard_before_subpix", canvas);
  }

  // 计算误差
  float cheeseboard_error_before_subpix = 0;
  for (size_t i = 0; i < corners_left.size(); i++) {
    cheeseboard_error_before_subpix +=
        std::fabs(corners_left[i].y - corners_right[i].y);
  }

  // 提取左图亚像素角点
  if (found_corner_left) {
    cv::TermCriteria tc = cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.01);
    // 亚像素检测
    cv::cornerSubPix(left_img, corners_left, cv::Size(5, 5), cv::Size(-1, -1),
                     tc);
    // drawChessboardCorners(left_img, patter_size, corners_left,
    //                       found_corner_left); // 画角点
  }

  // 提取右图亚像素角点
  if (found_corner_right) {
    cv::TermCriteria tc = cv::TermCriteria(
        cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.01);
    // 亚像素检测
    cv::cornerSubPix(right_img, corners_right, cv::Size(5, 5), cv::Size(-1, -1),
                     tc);
    // drawChessboardCorners(right_img, patter_size, corners_right,
    //                       found_corner_right);
  }

  // 计算误差
  float cheeseboard_error_after_subpix = 0;
  for (size_t i = 0; i < corners_left.size(); i++) {
    cheeseboard_error_after_subpix +=
        std::fabs(corners_left[i].y - corners_right[i].y);
  }
  std::cout << "cheeseboard error before/after subpix: "
            << cheeseboard_error_before_subpix / corners_left.size() << "/"
            << cheeseboard_error_after_subpix / corners_left.size()
            << std::endl;

  //  显示结果
  {
    Mat after_left_img, after_right_img;
    if (left_img.channels() > 1) {
      left_img.copyTo(after_left_img);
      right_img.copyTo(after_right_img);
    } else {
      cvtColor(left_img, after_left_img, CV_GRAY2RGB); //伪彩色图
      cvtColor(right_img, after_right_img, CV_GRAY2RGB);
    }

    for (size_t i = 0; i < corners_left.size(); i++) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                                    rng.uniform(0, 255));
      cv::circle(after_left_img, corners_left[i], 5, color, 0);
      cv::circle(after_right_img, corners_right[i], 5, color, 0);
    }

    Mat canvas;
    int w = after_left_img.cols;
    int h = after_left_img.rows;
    canvas.create(h, w * 2, CV_8UC3);
    after_left_img.copyTo(canvas(cv::Rect(0, 0, w, h)));
    after_right_img.copyTo(canvas(cv::Rect(w, 0, w, h)));

    cv::imshow("matched_for_epipolar_cheeseboard_after_subpix", canvas);
  }
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
void nudge_extrinsics(const cv::Mat &imleft_undistorted,
                      const cv::Mat &imright_undistorted, const Matrix3d &K_new,
                      const Matrix4d &right_T_left,
                      Matrix4d &optimized_right_T_left) {

  // Step-1 : Match point features from `imleft_undistorted,
  // imright_undistorted`
  MatrixXd u, ud; //< 3xN, ie. image co-ordinates represented in homogeneous
                  // cord system.
  point_feature_matches(imleft_undistorted, imright_undistorted, u, ud);
  cv::Mat dst;
  MiscUtils::plot_point_pair(imleft_undistorted, u, -1, imright_undistorted, ud,
                             -1, dst, 0); //, cv::Scalar(0,255,0));
  cv::imshow("gms_matches", dst);

  // Step-2 : In normalized co-ordinates : inv(K_new) * f' and inv(K_inv) * f
  MatrixXd f = K_new.inverse() * u;
  MatrixXd fd = K_new.inverse() * ud;

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
  cout << "CERES #residues : " << f.cols() << endl;
  for (int i = 0; i < f.cols(); i++) {
    int r = rand() % f.cols();
    CostFunction *cost_function =
        YonggenResidue::Create(f.col(r).head(3), fd.col(r).head(3));
    // problem.AddResidualBlock( cost_function, NULL, T_cap_q, T_cap_t );
    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.001),
                             T_cap_q, T_cap_t);
  }

  // 3.3 : Local Parameterization
  ceres::LocalParameterization *quaternion_parameterization =
      new ceres::QuaternionParameterization;
  // ceres::LocalParameterization *hv_parameterization = new
  // ceres::HomogeneousVectorParameterization(3);
  ceres::LocalParameterization *hv_parameterization =
      new UnitVectorParameterization();
  problem.SetParameterization(T_cap_q, quaternion_parameterization);
  problem.SetParameterization(T_cap_t, hv_parameterization);

  // Step-4 : Solve
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  // options.minimizer_type = ceres::LINE_SEARCH;
  // options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
  Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  // Step-5 : Retrive Solution
  Matrix4d T_cap;
  PoseManipUtils::raw_to_eigenmat(T_cap_q, T_cap_t, T_cap);
  cout << "CERES Solution : " << PoseManipUtils::prettyprintMatrix4d(T_cap)
       << endl;
  // cout << "CERES Solution T_cap_t: " << T_cap_t[0] << " " << T_cap_t[1] << "
  // " << T_cap_t[2] << endl;
  T_cap.col(3).topRows(3) *= n_norm;

  optimized_right_T_left = T_cap;
  cout << "CERES Solution T_cap(after rescaling): "
       << PoseManipUtils::prettyprintMatrix4d(T_cap) << endl;
}

// Stereo
int stereo_demo() {
  IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

#if false
    const std::string BASE = "/media/ha/Dataset/meituan/imuImg_calib_33003_20210615194513/image";
    std::string frame_id = "1623757535242051039";
    std::string left_image_path  = BASE + "/L" + frame_id + ".png";
    std::string right_image_path = BASE + "/R" + frame_id + ".png";

    // Abstract Camera
    camodocal::CameraPtr left_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
        "/media/ha/Dataset/meituan/imuImg_calib_33003_20210615194513/camera_left.yaml");
    camodocal::CameraPtr right_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
        "/media/ha/Dataset/meituan/imuImg_calib_33003_20210615194513/camera_right.yaml");

    // Extrinsics
    Matrix4d right_T_left;
    LoadExtrinsics("/media/ha/Dataset/meituan/imuImg_calib_33003_20210615194513/extrinsics.yaml", right_T_left);

#elif false
  const std::string BASE =
      "/home/bdca/stereo_calib_data/cam_imu_calib_data/33003_01/mav0";
  std::string frame_id = "1626682922795816103";
  std::string left_image_path = BASE + "/cam0/data/" + frame_id + ".png";
  std::string right_image_path = BASE + "/cam1/data/" + frame_id + ".png";

  // Abstract Camera
  camodocal::CameraPtr left_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/stereo_calib_data/cam_imu_calib_data/config/"
          "camera_left.yaml");
  camodocal::CameraPtr right_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/stereo_calib_data/cam_imu_calib_data/config/"
          "camera_right.yaml");

  // Extrinsics
  Matrix4d right_T_left;
  LoadExtrinsics(
      "/home/bdca/stereo_calib_data/cam_imu_calib_data/config/extrinsics.yaml",
      right_T_left);

#elif true
  const std::string BASE =
      "//home/bdca/dataset/calibration/cam_checkerboard/mav0";
  std::string frame_id = "1403709043137837056";
  std::string left_image_path = BASE + "/cam0/data/" + frame_id + ".png";
  std::string right_image_path = BASE + "/cam1/data/" + frame_id + ".png";

  // Abstract Camera
  camodocal::CameraPtr left_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/dataset/calibration/cam_checkerboard/camera_left.yaml");
  camodocal::CameraPtr right_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/dataset/calibration/cam_checkerboard/camera_right.yaml");

  // Extrinsics
  Matrix4d right_T_left;
  LoadExtrinsics("/home/bdca/dataset/calibration/cam_checkerboard/"
                 "extrinsics_groundtruth.yaml",
                 right_T_left);
                 
#elif false
  const std::string BASE =
      "/home/bdca/workspace/stereo-calibration/calib_imgs/1";

  std::string frame_id = "1";
  std::string left_image_path = BASE + "/left" + frame_id + ".jpg";
  std::string right_image_path = BASE + "/right" + frame_id + ".jpg";

  // Abstract Camera
  camodocal::CameraPtr left_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/workspace/stereo-calibration/config/seq1/"
          "camera_left.yaml");
  camodocal::CameraPtr right_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/workspace/stereo-calibration/config/seq1/"
          "camera_right.yaml");

  // Extrinsics
  Matrix4d right_T_left;
  LoadExtrinsics(
      "/home/bdca/workspace/stereo-calibration/config/seq1/extrinsics.yaml",
      right_T_left);

#elif false
  const std::string BASE = "/home/bdca/dataset/MH_01_easy/mav0";
  std::string frame_id = "1403636670163555584";
  std::string left_image_path = BASE + "/cam0/data/" + frame_id + ".png";
  std::string right_image_path = BASE + "/cam1/data/" + frame_id + ".png";

  // Abstract Camera
  camodocal::CameraPtr left_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/dataset/V1_01_easy/camera_left.yaml");
  camodocal::CameraPtr right_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/dataset/V1_01_easy/camera_right.yaml");

  // Extrinsics
  Matrix4d right_T_left;
  LoadExtrinsics("/home/bdca/dataset/V1_01_easy/extrinsics.yaml", right_T_left);
#endif

  cout << left_camera->parametersToString() << endl;
  cout << right_camera->parametersToString() << endl;

  cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d(right_T_left)
       << endl;
  cout << "right_T_left=\n" << right_T_left.format(numpyFmt) << endl;

#if 0
  // add noise
  Matrix4d delta;
  PoseManipUtils::rawyprt_to_eigenmat(Vector3d(g_noise_r, g_noise_r, g_noise_r),
                                      Vector3d(g_noise_r, g_noise_r, g_noise_r),
                                      delta);
  right_T_left = (delta * right_T_left).eval();
  cout << "right_T_left(after applying delta): "
       << PoseManipUtils::prettyprintMatrix4d(right_T_left) << endl;
#endif

  std::shared_ptr<StereoGeometry> stereogeom;
  stereogeom =
      std::make_shared<StereoGeometry>(left_camera, right_camera, right_T_left);

  Eigen::Matrix3d new_cam_K;
  GeometryUtils::getK(left_camera, new_cam_K);
  stereogeom->set_K(new_cam_K);

  // Raw Image - Image from camera
  cv::Mat imleft_raw = cv::imread(left_image_path, 0);
  cv::Mat imright_raw = cv::imread(right_image_path, 0);

  // Undistort Only
  cv::Mat imleft_undistorted, imright_undistorted;
  stereogeom->do_image_undistortion(imleft_raw, imright_raw, imleft_undistorted,
                                    imright_undistorted);

  cout << " ======= before opt extrinsics ======= \n";
  {
    cv::Mat imleft_srectified, imright_srectified;
    stereogeom->do_stereo_rectification_of_undistorted_images(
        imleft_undistorted, imright_undistorted, imleft_srectified,
        imright_srectified);
    print_epipolar_error(imleft_srectified, imright_srectified);
  }

  // Stereo calibration
  {
    Matrix4d optimized_right_T_left;
    nudge_extrinsics(imleft_undistorted, imright_undistorted,
                     stereogeom->get_K(), right_T_left, optimized_right_T_left);
    cout << "optimized_right_T_left: "
         << PoseManipUtils::prettyprintMatrix4d(optimized_right_T_left) << endl;
    stereogeom->set_stereoextrinsic(optimized_right_T_left);
  }

  // will get 3d points, stereo-rectified image, and disparity false colormap
  MatrixXd _3dpts; // 4xN
  cv::Mat imleft_srectified, imright_srectified;
  cv::Mat disparity_for_visualization;
  stereogeom->get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(
      imleft_raw, imright_raw, imleft_srectified, imright_srectified, _3dpts,
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

int main() { 
  stereo_demo(); 
}
