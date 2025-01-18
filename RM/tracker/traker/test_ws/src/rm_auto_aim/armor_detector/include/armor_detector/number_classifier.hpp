#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
public:
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);// src: 输入的源图像 armors: 用于存储检测到的装甲板信息的向量

  void classify(std::vector<Armor> & armors);// 对装甲板进行分类 armors: 包含待分类装甲板的向量

  double threshold; // 分类阈值

private:
  cv::dnn::Net net_;// 神经网络模型
  std::vector<std::string> class_names_;// 类别名称列表
  std::vector<std::string> ignore_classes_;// 需要忽略的类
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
