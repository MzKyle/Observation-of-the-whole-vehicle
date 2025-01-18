#ifndef ARMOR_DETECTOR__ARMOR_HPP_
#define ARMOR_DETECTOR__ARMOR_HPP_

#include <opencv2/core.hpp>

// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID };
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

//接受一个 cv::RotatedRect 对象，计算并初始化灯条的顶部、底部、长度、宽度和倾斜角度。
struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];// 声明一个包含四个点的数组
    box.points(p);// 获取旋转矩形的四个顶点
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });// 按照 y 坐标对这四个点进行排序 确保顶点的顺序是固定的
    top = (p[0] + p[1]) / 2;// 计算顶部和底部的中心点
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);// 计算灯的长度（顶部中心点到底部中心点的距离）
    width = cv::norm(p[0] - p[1]);// 计算灯的宽度（左侧两个顶点之间的距离）

// 计算灯的倾斜角度（以度为单位）
    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color; //颜色
  cv::Point2f top, bottom;//顶部和底部的中心点
  double length;//长
  double width;//宽
  float tilt_angle;//倾斜角
};

struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    // 根据 Light 对象的中心 x 坐标进行比较，确定左右灯条
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;// 计算装甲板的中心点
  }

  // Light pairs part
  Light left_light, right_light; //左右灯条
  cv::Point2f center;//装甲板中心点
  ArmorType type;//装甲板类型

  // Number part
  cv::Mat number_img; //数字图像
  std::string number; //识别出的数字字符串
  float confidence; //识别置信度 ，通常是一个介于0到1之间的浮点数
  std::string classfication_result; //分类结果
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
