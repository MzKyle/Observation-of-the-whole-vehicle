#ifndef ARMOR_DETECTOR__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class PnPSolver
{
public:
// 构造函数，初始化相机矩阵和畸变系数
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position  解决PnP问题，获取3D位置
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center  计算装甲板中心与图像中心的距离
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_; // 相机内参矩阵
  cv::Mat dist_coeffs_;  // 畸变系数

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;// 小装甲板的宽度
  static constexpr float SMALL_ARMOR_HEIGHT = 55;// 小装甲板的高度
  static constexpr float LARGE_ARMOR_WIDTH = 225;// 大装甲板的宽度
  static constexpr float LARGE_ARMOR_HEIGHT = 55;// 大装甲板的高度

  // Four vertices of armor in 3d  // 装甲板在3D空间中的四个顶点
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__PNP_SOLVER_HPP_
