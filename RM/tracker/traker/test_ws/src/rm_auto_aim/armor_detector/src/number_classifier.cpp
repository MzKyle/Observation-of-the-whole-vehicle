// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"

namespace rm_auto_aim
{
  //model_path：ONNX模型文件的路径。
  //label_path：标签文件的路径，文件中包含每个类别的名称。
  //thre：分类阈值，用于过滤低置信度的分类结果。
  //ignore_classes：需要忽略的类别名称列表
NumberClassifier::NumberClassifier(//构造函数
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)
{
  net_ = cv::dnn::readNetFromONNX(model_path);//读取并加载ONNX模型

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);//从标签文件中读取类别名称，并存储在class_names_向量中
  }
}

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)//提取数字所在的ROI图像存储在armor.number_img
{
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);

  for (auto & armor : armors) {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;
  }
}

void NumberClassifier::classify(std::vector<Armor> & armors)
{
  for (auto & armor : armors) {
    cv::Mat image = armor.number_img.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image 创建神经网络输入所需的blob
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

//将blob输入到神经网络中进行前向传播，得到输出
    // Set the input blob for the neural network
    net_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax  进行softmax处理，以获得每个类别的概率分布
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

 //根据最高概率确定分类结果，并计算置信度
    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

// 将分类结果和置信度存储在armor对象中
    armor.confidence = confidence;
    armor.number = class_names_[label_id];

//将装甲板的数字识别结果和置信度格式化为一个字符串，并将该字符串存储在armor.classfication_result中
    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();
  }


//通过三个条件（置信度、忽略类和类型不匹配）筛选出需要移除的装甲板对象，然后从 armors 容器中删除这些对象
  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) {
        if (armor.confidence < threshold) {//置信度检查 armor 的置信度小于阈值 threshold，则返回 true，表示这个元素应该被移除
          return true;
        }

        for (const auto & ignore_class : ignore_classes_) {//忽略类检查 遍历 ignore_classes_ 容器，如果 armor 的数字与任何一个忽略类匹配，则返回 true，表示这个元素应该被移除
          if (armor.number == ignore_class) {
            return true;
          }
        }

        bool mismatch_armor_type = false; //类型不匹配检查
        if (armor.type == ArmorType::LARGE) {
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == ArmorType::SMALL) {
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
}

}  // namespace rm_auto_aim
