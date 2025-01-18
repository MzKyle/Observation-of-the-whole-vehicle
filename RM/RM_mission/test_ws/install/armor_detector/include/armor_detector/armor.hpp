// Copyright 2022 Chen Jun
// Licensed under the MIT License.

/*cv::RotatedRect是OpenCV库中的一个类，用于表示一个旋转矩形。这个类主要用于存储旋转矩形的信息，如中心点、宽度、高度、旋转角度等，并提供了一组方法来处理和操作这些信息。

以下是cv::RotatedRect的一些主要属性和方法：

构造函数：可以通过提供中心点、宽度、高度和旋转角度来创建一个cv::RotatedRect对象。

center：表示旋转矩形的中心点，是一个cv::Point2f类型的值。

size：表示旋转矩形的宽度和高度，是一个cv::Size2f类型的值。

angle：表示旋转矩形的旋转角度，以度为单位。

points：返回旋转矩形的四个顶点坐标，存储在一个cv::Point2f类型的数组中。

boundingRect：返回旋转矩形的最小外接矩形，是一个cv::Rect类型的值。

contains：判断一个给定的点是否在旋转矩形内。

intersects：判断两个旋转矩形是否相交。

area：计算旋转矩形的面积。

get_rotation_matrix2D：获取旋转矩阵，用于将旋转矩形变换到原始位置。

get_rect_points：获取旋转矩形的四个顶点坐标。

get_rect_center：获取旋转矩形的中心点。

get_rect_size：获取旋转矩形的宽度和高度。

get_rect_angle：获取旋转矩形的旋转角度。

通过使用cv::RotatedRect类，可以方便地处理和操作旋转矩形，例如计算面积、判断点是否在矩形内、获取顶点坐标等。*/
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

enum class ArmorType { SMALL, LARGE, INVALID };//小型，大型，无效
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

//表示装甲板上的一个灯条
struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

struct Armor
{
  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img;
  std::string number;
  float confidence;
  std::string classfication_result;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
