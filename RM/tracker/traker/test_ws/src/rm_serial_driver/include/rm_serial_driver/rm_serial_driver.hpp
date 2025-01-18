#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <iomanip>


#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/send.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"

//创建一个 ROS2 节点，通过串行端口与外部设备进行通信，接收目标数据并发布相应的关节状态和速度信息，同时支持参数设置和重置跟踪器等功能
namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;
  float pitch_trans(float originAngle);//角度转换函数
  float pitch_re_trans(float originAngle);
  float yaw_trans(float originAngle);
  float yaw_re_trans(float originAngle);
private:
  // 在 RMSerialDriver 类的头文件中添加成员变量
  std::ofstream csv_file_;

  void getParams();//获取参数

  void receiveData();//接收数据

  void sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg);//发送数据

  void reopenPort();//新打开串行端口

  void setParam(const rclcpp::Parameter & param);//设置参数

  void resetTracker();//重置跟踪器

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;//智能指针 owned_ctx_，用于管理 I/O 上下文对象
  std::string device_name_;//存储串行设备的名称
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;//智能指针 device_config_，用于管理串行端口配置对象
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;//智能指针 serial_driver_，用于管理串行驱动对象

  // Param client to set detect_colr 参数客户端设置检测颜色
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;//共享的未来对象，用于异步获取参数设置结果
  bool initial_set_param_ = false;  //表示是否已经进行了初始参数设置
  uint8_t previous_receive_color_ = 0;  //无符号 8 位整数，用于存储之前接收到的颜色
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;  //共享指针，指向异步参数客户端
  ResultFuturePtr set_param_future_; //共享未来对象，用于存储参数设置的结果

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_; //管理重置跟踪器服务客户端

  // Aimimg point receiving from serial port for visualization 从串行端口接收的瞄准点进行可视化
  visualization_msgs::msg::Marker aiming_point_;

  double timestamp_offset_ = 0; //时间戳偏移量
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;//共享指针，指向关节状态发布者
  rclcpp::Publisher<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_pub_;//速度发布者
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;//目标订阅者
  rclcpp::Subscription<auto_aim_interfaces::msg::Send>::SharedPtr send_sub_;//发送订阅者

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;//共享指针，指向延迟发布者
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;//标记发布者

  std::thread receive_thread_;//线程对象 receive_thread_，用于处理接收数据的任务
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
