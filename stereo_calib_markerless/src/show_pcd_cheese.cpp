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

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace ceres;

float g_noise_r = 1;   // 旋转扰动值
float g_noise_t = 0;   // 平移扰动值
int g_corner_row = 10; // 棋盘格行
int g_corner_col = 8;  // 棋盘格列

pcl::PointCloud<pcl::PointXYZ>::Ptr
    source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

// 生成点云
void GenerateCloud(const cv::Mat& disparity, int size, int w, int h, MatrixXd _3dpts,
                   double fx, double fy, double cx, double cy, double b) {

  source_cloud->width = w;
  source_cloud->height = h;
  // source_cloud->is_dense = false;
  source_cloud->points.resize(source_cloud->width * source_cloud->height);

  cout << "disparity.type() " << disparity.type() << endl;
  cout << "channel " << disparity.channels() << endl;

  int i = 0;
  for (int v = 0; v < h; v++) {
    for (int u = 0; u < w; u++) {
      unsigned int disp = disparity.ptr<unsigned short>(v)[u];
    // short disp = disparity.at<short>(v, u);

      if (disp <= 0) {
        // cout << "d=0" << endl;
        continue;
      }

      double d = (double)disp /16.;

      Vector3d point(0, 0, 0);
      point[2] = (fx * b ) / d;
      point[1] = (v - cy) * point[2] / fy;
      point[0] = (u - cx) * point[2] / fx;
      source_cloud->points[i].x = point[0];
      source_cloud->points[i].y = point[1];
      source_cloud->points[i].z = point[2];

      i++;
    }
  }
  cout << source_cloud->size() << endl;

  pcl::io::savePCDFileASCII(
      "/home/bdca/workspace/catkin_ws/src/cheese_cloud.pcd", *source_cloud);
}

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


// Stereo
int stereo_demo() {
  IOFormat numpyFmt(FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");

  const std::string BASE = "/home/bdca/dataset/stereo_images/record_3";
  std::string frame_id = "1637573430036735967";
  std::string left_image_path = BASE + "/cam0/" + frame_id + ".png";
  std::string right_image_path = BASE + "/cam1/" + frame_id + ".png";

  // Abstract Camera
  camodocal::CameraPtr left_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/dataset/stereo_images/camera_left.yaml");
  camodocal::CameraPtr right_camera =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
          "/home/bdca/dataset/stereo_images/camera_right.yaml");

  // Extrinsics
  Matrix4d right_T_left;
  LoadExtrinsics("/home/bdca/dataset/stereo_images/opt_extrinsic.yaml",
                 right_T_left);

  cv::FileStorage fs("/home/bdca/dataset/stereo_images/camera_left.yaml",
                     cv::FileStorage::READ);
  cv::FileStorage fs1("/home/bdca/dataset/stereo_images/opt_extrinsic.yaml",
                      cv::FileStorage::READ);

  cout << left_camera->parametersToString() << endl;
  cout << right_camera->parametersToString() << endl;

  cout << "right_T_left: " << PoseManipUtils::prettyprintMatrix4d(right_T_left)
       << endl;
  cout << "right_T_left=\n" << right_T_left.format(numpyFmt) << endl;

  std::shared_ptr<StereoGeometry> stereogeom;
  stereogeom =
      std::make_shared<StereoGeometry>(left_camera, right_camera, right_T_left);

  Eigen::Matrix3d new_cam_K;
  GeometryUtils::getK(left_camera, new_cam_K);
  stereogeom->set_K(new_cam_K);

  // Raw Image - Image from camera
  cv::Mat imleft_raw = cv::imread(left_image_path, 0);
  cv::Mat imright_raw = cv::imread(right_image_path, 0);

  cv::Mat disp_raw;
  stereogeom->do_stereoblockmatching_of_raw_images(imleft_raw, imright_raw,
                                                   disp_raw);

  // will get 3d points, stereo-rectified image, and disparity false colormap
  MatrixXd _3dpts; // 4xN
  cv::Mat imleft_srectified, imright_srectified;
  cv::Mat disparity_for_visualization;
  stereogeom->get_srectifiedim_and_3dpoints_and_disparity_from_raw_images(
      imleft_raw, imright_raw, imleft_srectified, imright_srectified, _3dpts,
      disparity_for_visualization);
  int col = disp_raw.cols;
  int row = disp_raw.rows;

  // 读取相机参数
  double fx, fy, cx, cy, b;

  cv::FileNode node = fs["projection_parameters"];
  cv::FileNode node1 = fs1["transform"];
  node["fx"] >> fx;
  node["fy"] >> fy;
  node["cx"] >> cx;
  node["cy"] >> cy;
  b = static_cast<double>(node1["t_x"]);
  cout << fx << " " << fy << " " << cx << " " << cy << " " << b << endl;
  // 显示点云
  int size = _3dpts.cols();
  GenerateCloud(disp_raw, size, col, row, _3dpts, fx, fy, cx, cy, -b);

  cv::imshow("disparity_for_visualization", disparity_for_visualization);

  cv::waitKey(0);

  ros::NodeHandle nh;
  ros::Publisher pubCloud =
      nh.advertise<sensor_msgs::PointCloud2>("target_cloud", 1);
  sensor_msgs::PointCloud2 target_cloud;
  // 转换点云格式  pcl->ros
  pcl::toROSMsg(*source_cloud, target_cloud);
  target_cloud.header.frame_id = "map";

  ros::Rate rate(1);
  while (ros::ok()) {
    // 发布
    pubCloud.publish(target_cloud);
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "show_pcd");
  stereo_demo();
}
