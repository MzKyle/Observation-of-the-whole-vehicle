// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/detector_node.hpp"

namespace rm_auto_aim
{
//构造函数，接受一个NodeOptions对象作为参数
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
: Node("armor_detector", options)//用基类Node的构造函数，初始化节点名称为armor_detector
{
  RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");

  // init Detector
  detector_ = initDetector();

  // Armors Publisher 创建装甲板发布者，发布到"/detector/armors"话题，使用SensorDataQoS
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher 配置可视化标记的属性
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";  //命名空间
  armor_marker_.action = visualization_msgs::msg::Marker::ADD; //动作类型
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;  //形状
  armor_marker_.scale.x = 0.05; //尺寸
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;//颜色
  armor_marker_.color.g = 0.5;
  armor_marker_.color.b = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1); //生命周期

// 配置文本标记的属性
  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

// 创建标记发布者，发布到"/detector/marker"话题，队列大小为10
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/detector/marker", 10);

  // Debug Publishers  检查是否启用调试模式，如果启用则创建调试发布者
  debug_ = this->declare_parameter("debug", false);
  if (debug_) {
    createDebugPublishers();
  }

  // Debug param change moniter
  //创建一个 rclcpp::ParameterEventHandler 对象，用于处理参数变化事件
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  //添加一个回调函数，当参数 "debug" 发生变化时调用
  debug_cb_handle_ =
    debug_param_sub_->add_parameter_callback("debug", [this](const rclcpp::Parameter & p) {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    });

//订阅相机信息话题 /camera_info，使用传感器数据质量服务（QoS）。
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);//提取相机中心点坐标并存储在 cam_center_ 中
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);//将相机信息消息保存到 cam_info_ 中
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);//使用相机内参和畸变系数创建 PnP 求解器 pnp_solver_
      cam_info_sub_.reset();//重置相机信息订阅对象 cam_info_sub_，表示不再需要继续订阅
    });

//订阅图像话题 /image_raw，使用传感器数据质量服务（QoS）
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));//当接收到图像消息时，调用 ArmorDetectorNode 类的 imageCallback 方法进行处理。
}

// 定义 imageCallback 方法，当接收到图像消息时调用
void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  auto armors = detectArmors(img_msg);//调用 detectArmors 方法检测装甲板，返回检测结果 armors

  if (pnp_solver_ != nullptr) {//如果 PnP 求解器已初始化
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;//设置消息头为当前图像消息的头
    armors_msg_.armors.clear();//清空之前的消息内容
    marker_array_.markers.clear();
    armor_marker_.id = 0;//重置标记 ID
    text_marker_.id = 0;


//遍历检测到的每个装甲板
    auto_aim_interfaces::msg::Armor armor_msg;
    for (const auto & armor : armors) {
      cv::Mat rvec, tvec;
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);//使用 PnP 求解器计算旋转向量和平移向量，结果存储在 rvec 和 tvec 中
      if (success) {
        // Fill basic info 填充基本信息，包括类型和编号
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // Fill pose 填充位置信息，包括 x、y、z 坐标
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // rvec to 3x3 rotation matrix 将旋转向量转换为旋转矩阵
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // rotation matrix to quaternion 转换为四元数，并赋值给姿态的四元数部分
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // Fill the distance to image center 计算装甲板中心到图像中心的距离，并填充到消息中
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        // Fill the markers
        armor_marker_.id++;//更新标记 ID，并根据装甲板类型设置标记的缩放比例
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23; //设置标记的姿态和文本内容
        armor_marker_.pose = armor_msg.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        armors_msg_.armors.emplace_back(armor_msg);//将装甲板信息和标记添加到相应的消息中
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      } else {//如果 PnP 求解失败，记录警告日志
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // Publishing detected armors发布检测到的装甲板信息
    armors_pub_->publish(armors_msg_);

    // Publishing marker 发布可视化标记
    publishMarkers();
  }
}

std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  //创建一个参数描述符 param_desc，并设置其整数范围属性
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);//默认值为 160，范围为 0 到 255。

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = declare_parameter("detect_color", RED, param_desc);//默认值为 RED，范围为 0 到 1。


//创建并初始化 Detector::LightParams 结构体 l_params，包含最小比例、最大比例和最大角度参数
  Detector::LightParams l_params = {
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

//使用上述参数创建一个新的 Detector 对象，并将其封装在 std::unique_ptr 中。
  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // Init classifier
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector"); //获取包路径，并构建模型文件和标签文件的路径。
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);  //声明分类器的阈值参数，默认值为 0.7。
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"}); //声明忽略类别参数，默认值为包含 "negative" 的向量。
  //使用模型路径、标签路径、阈值和忽略类别初始化 NumberClassifier，并将其赋值给 detector 的 classifier 成员。
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector; //返回初始化好的 Detector 对象
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
  const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // Convert ROS img to cv::Mat
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image; //将传入的 ROS 图像消息转换为 OpenCV 的 cv::Mat 格式

  // Update params   更新检测器的二值化阈值、检测颜色和分类器阈值参数
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->detect_color = get_parameter("detect_color").as_int();
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  auto armors = detector_->detect(img); //调用检测器的 detect 方法进行装甲板检测，返回检测结果

  auto final_time = this->now(); //计算处理延迟时间，并记录调试日志
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  // Publish debug info
  if (debug_) {// 如果处于调试模式，则执行以下代码块
    binary_img_pub_.publish(// 将二值化后的图像发布出去，使用"mono8"编码格式
      cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img).toImageMsg());

    // Sort lights and armors data by x coordinate 根据x坐标对灯光和装甲板数据进行排序
    std::sort(
      detector_->debug_lights.data.begin(), detector_->debug_lights.data.end(),
      [](const auto & l1, const auto & l2) { return l1.center_x < l2.center_x; });
    std::sort(
      detector_->debug_armors.data.begin(), detector_->debug_armors.data.end(),
      [](const auto & a1, const auto & a2) { return a1.center_x < a2.center_x; });
// 使用lambda表达式根据中心点的x坐标进行排序

    lights_data_pub_->publish(detector_->debug_lights); // 发布灯光的调试信息
    armors_data_pub_->publish(detector_->debug_armors);// 发布装甲板的调试信息

    if (!armors.empty()) {// 如果检测到装甲板
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(// 获取所有数字图像
        *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img).toImageMsg());
    }// 发布包含所有数字的图像，使用"mono8"编码格式

    detector_->drawResults(img);//在原始图像上绘制检测结果
    // Draw camera center
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);//绘制相机中心点 用蓝色圆圈标记相机中心点，半径为5，线条宽度为2
    // Draw latency  绘制延迟时间
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(// 在图像上绘制延迟时间，位置为(10, 30)，字体为Hershey简单字体，大小为1.0，颜色为绿色，线条宽度为2
      img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());// 发布最终结果图像，使用"rgb8"编码格式
  }

  return armors; // 返回检测到的装甲板列表
}


//创建两个发布者，分别用于发布灯光和装甲板的调试信息
void ArmorDetectorNode::createDebugPublishers()
{
  lights_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugLights>("/detector/debug_lights", 10);
  armors_data_pub_ =
    this->create_publisher<auto_aim_interfaces::msg::DebugArmors>("/detector/debug_armors", 10);

//创建三个图像传输发布者，分别用于发布二值化图像、数字图像和结果图像
  binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
  number_img_pub_ = image_transport::create_publisher(this, "/detector/number_img");
  result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
}

//重置灯光和装甲板调试信息的发布者
void ArmorDetectorNode::destroyDebugPublishers()
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();//关闭所有图像传输发布者
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

//根据是否检测到装甲板来设置标记的动作（添加或删除），然后发布标记信息
void ArmorDetectorNode::publishMarkers()
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
//注册组件宏这个宏使得当前节点可以被加载到运行中的进程中，使其成为可发现和可加载的组件
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
