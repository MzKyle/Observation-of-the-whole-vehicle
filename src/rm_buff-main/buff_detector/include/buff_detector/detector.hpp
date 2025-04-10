#ifndef BUFF_DETECTOR__DETECTOR_HPP_
#define BUFF_DETECTOR__DETECTOR_HPP_

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "buff_detector/blade.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "openvino/openvino.hpp"

#define KPT_NUM 5
#define CLS_NUM 4

namespace rm_buff
{
class Detector
{
public:
  // Detector();
  Detector(const std::string model_path);
  // ~Detector();
  void ProcessVideo(const std::string& video_path);
  std::vector<Blade> Detect(cv::Mat & image);

  void draw_blade(cv::Mat & img);

  // param
  double nms_threshold;  //非极大值抑制（NMS）的阈值
  double conf_threshold;  //置信度阈值，用于过滤低置信度的检测结果
  double image_size;   //输入图像的大小
  double bin_threshold;   //二值化阈值
  double fault_tolerance_ratio;  //容错率

  // visual
  cv::Mat binary_img;
  cv::Mat debug_img;  //调试图像

private:
  std::string model_path_;  //存储深度学习模型的路径
  std::string bin_path_;  //存储模型的二进制文件路径

  ov::Core core_;  //OpenVINO 核心对象，用于管理设备和模型
  std::shared_ptr<ov::Model> model_;  //指向 OpenVINO 模型的共享指针
  ov::CompiledModel compiled_model_;  //编译后的模型
  ov::InferRequest infer_request_;  //推理请求对象，用于执行模型推理
  ov::Tensor input_tensor_;  //输入张量，用于存储输入数据

  int padd_w_ = 0;  //分别存储图像填充的宽度和高度
  int padd_h_ = 0;

  std::vector<Blade> blade_array_;


  //存储装甲板、角点和关键点的模板点
  std::vector<cv::Point2f> blade_template_;
  std::vector<cv::Point2f> corner_template_;
  std::vector<cv::Point2f> kpt_template_;

  bool calibrate_kpts(Blade & blade, cv::Mat & img);  //校准装甲板的关键点，返回校准是否成功

  cv::Mat letterbox(cv::Mat & src, int h, int w);  //对输入图像进行填充和缩放，使其符合模型输入要求

  void non_max_suppression(//执行非极大值抑制，过滤重叠的检测框
    ov::Tensor & output, float conf_thres, float iou_thres, int nc, cv::Size img_size);

  const std::vector<std::string> class_names = {"RR", "RW", "BR", "BW"};
};
}  // namespace rm_buff

#endif  // BUFF_DETECTOR__DETECTOR_HPP_
