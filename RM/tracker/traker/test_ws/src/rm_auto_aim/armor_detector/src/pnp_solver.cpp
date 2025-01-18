// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
  //camera_matrix：相机内参矩阵，包含焦距和主点坐标     dist_coeffs：相机的畸变系数
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()), //将传入的相机内参数组转换为OpenCV的cv::Mat类型，并克隆一份数据存储在成员变量camera_matrix_中
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()) //将畸变系数数组转换为cv::Mat类型，并克隆一份数据存储在成员变量dist_coeffs_中
{
  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}


//armor：包含装甲板的相关信息，包括其类型和四个角点的图像坐标。rvec：输出的旋转向量。tvec：输出的平移向量
bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  std::vector<cv::Point2f> image_armor_points;

  // Fill in image points
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  // Solve pnp
  auto object_points = armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;//根据装甲板的类型（小或大），选择相应的模型点集合
  return cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false, //使用选定的模型点和图像点，以及相机内参和畸变系数，求解PnP问题，得到旋转向量rvec和平移向量tvec。
    cv::SOLVEPNP_IPPE);
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)//计算图像点到图像中心的距离  从相机内参矩阵中提取主点坐标（cx, cy）
{
  float cx = camera_matrix_.at<double>(0, 2);//计算该图像点到主点的距离，并返回结果
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
