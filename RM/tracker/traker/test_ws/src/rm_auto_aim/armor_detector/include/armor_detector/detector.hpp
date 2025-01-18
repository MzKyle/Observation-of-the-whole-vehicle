// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
class Detector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio; // 灯条的最小宽高比
    double max_ratio; // 灯条的最大宽高比
    // vertical angle
    double max_angle;  // 灯条的最大垂直角度
  };

  struct ArmorParams
  {
    double min_light_ratio;  // 两个灯条之间的最小比例
    // light pairs distance
    double min_small_center_distance;  // 小装甲板中心之间的最小距离
    double max_small_center_distance;  // 小装甲板中心之间的最大距离
    double min_large_center_distance;  // 大装甲板中心之间的最小距离
    double max_large_center_distance;  // 大装甲板中心之间的最大距离
    // horizontal angle
    double max_angle; // 灯条之间的最大水平角度
  };
// 构造函数，初始化检测器，参数包括二值化阈值、颜色、灯条参数和装甲板参数
  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

// 检测输入图像中的装甲板，返回检测到的装甲板列表
  std::vector<Armor> detect(const cv::Mat & input);

// 预处理输入图像，返回处理后的图像
  cv::Mat preprocessImage(const cv::Mat & input);
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);// 在RGB图像和二值图像中查找灯条，返回找到的灯条列表
  std::vector<Armor> matchLights(const std::vector<Light> & lights);// 根据灯条匹配装甲板，返回匹配到的装甲板列表

  // For debug usage
  cv::Mat getAllNumbersImage(); // 获取所有数字图像，用于调试
  void drawResults(cv::Mat & img); // 在图像上绘制检测结果，用于调试

  int binary_thres;
  int detect_color;
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;// 数字分类器，用于识别装甲板上的数字

  // Debug msgs
  cv::Mat binary_img;
  auto_aim_interfaces::msg::DebugLights debug_lights; // 调试用的灯条信息
  auto_aim_interfaces::msg::DebugArmors debug_armors; // 调试用的装甲板信息

private:
  bool isLight(const Light & possible_light); // 判断一个可能的灯条是否为有效灯条
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);// 判断两个灯条之间是否包含其他灯条
  ArmorType isArmor(const Light & light_1, const Light & light_2);// 根据两个灯条判断是否为装甲板，并返回装甲板类型

  std::vector<Light> lights_;// 存储找到的灯条列表
  std::vector<Armor> armors_;// 存储匹配到的装甲板列表
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
