// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/rm_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},//创建一个新的I/O上下文对象，用于管理异步操作
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}//创建一个串行驱动对象，并将其与I/O上下文关联。
{

  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // Create Publisher
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));//关节状态
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);//延迟
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);//标记
  velocity_pub_ = this->create_publisher<auto_aim_interfaces::msg::Velocity>("/current_velocity", 10);//速度

  // Detect parameter client 创建一个异步参数客户端，用于与 armor_detector 节点进行参数通信
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client  创建一个服务客户端，用于调用 /tracker/reset 服务。
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);//初始化串行端口，使用设备名称和配置
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();//如果端口未打开，则打开端口并启动接收数据的线程
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

//可视化标记
  aiming_point_.header.frame_id = "odom";//参考坐标系为"odom"
  aiming_point_.ns = "aiming_point";//命名空间设置为"aiming_point"
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;//标记类型设置为球体（SPHERE）
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;//动作设置为添加（ADD）
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;//标记的大小，x、y、z方向都为0.12
  aiming_point_.color.r = 1.0;//标记的颜色为白色（RGBA值均为1.0）
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);//标记的生存时间为0.1秒

  // Create Subscription
  /*create_subscription: 创建一个订阅者，订阅主题为"/tracker/send"。
auto_aim_interfaces::msg::Send: 订阅的消息类型为auto_aim_interfaces::msg::Send。
rclcpp::SensorDataQoS(): 使用传感器数据质量服务（QoS）配置。
std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1): 绑定回调函数sendData，当接收到消息时调用该函数*/
  send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
    "/tracker/send", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));


  // target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/target", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{

}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);//大小为一的字节向量，就接收一个包头
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) { //修改
    try {
      serial_driver_->port()->receive(header);//从串口接收一个字节的数据到 header

      if (header[0] == 0x5A) {//接收到的头部字节是 0x5A，表示这是一个有效的数据包
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);//将头部字节插入到 data 的开头
        ReceivePacket packet = fromVector(data); //将字节向量转换为 ReceivePacket 结构体

        bool crc_ok =
           crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));//CRC16 算法验证数据包的完整性。如果 CRC 校验通过，则 crc_ok 为 true

        if (crc_ok) {
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {//如果 CRC 校验通过，并且检测颜色发生变化或初始参数未设置，则更新参数
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {//如果数据包要求重置追踪器，则调用 resetTracker() 函数
            resetTracker();
          }

          //// 打印 data 结构体中的 xyz 和 yaw 值
          std::cout << "xyz: (" << packet.aim_x << ", " << packet.aim_y << ", " << packet.aim_z << ")" << std::endl;
          std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;
                    RCLCPP_INFO(get_logger(), "CRC OK!");

          ////LOG [Receive] aim_xyz
  
          RCLCPP_INFO(get_logger(), "[Receive] aim_x %f!", packet.aim_x);
          RCLCPP_INFO(get_logger(), "[Receive] aim_y %f!", packet.aim_y);
          RCLCPP_INFO(get_logger(), "[Receive] aim_z %f!", packet.aim_z);

          // //LOG [Receive] [Receive] rpy
          RCLCPP_INFO(get_logger(), "[Receive] roll %f!", packet.roll);
          RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", packet.pitch);
          RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", packet.yaw);

          sensor_msgs::msg::JointState joint_state;//创建并填充 JointState 消息
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          joint_state.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");

          //float temp_pitch = pitch_re_trans(packet.pitch);
          joint_state.position.push_back(packet.pitch);

          //float temp_yaw = yaw_re_trans(packet.yaw);
          joint_state.position.push_back(packet.yaw);
          joint_state_pub_->publish(joint_state);

          auto_aim_interfaces::msg::Velocity current_velocity;//创建并填充 Velocity 消息
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          current_velocity.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          current_velocity.velocity = packet.current_v;
          velocity_pub_->publish(current_velocity);

          if (abs(packet.aim_x) > 0.01) {//瞄准点的 x 坐标绝对值大于 0.01，则创建并填充 aiming_point_ 消息，包括时间戳和位置，然后发布该消息
            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = packet.aim_x;
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_);
          }
        } else {//如果 CRC 校验失败，记录错误日志
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {//如果接收到的头部字节不是 0x5A，记录警告日志，但每20秒只记录一次。
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {//捕获所有异常，记录错误日志，并尝试重新打开串口
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{//定义了一个静态的映射表 id_unit8_map，将字符串映射到 uint8_t 类型的值
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    packet.is_fire = msg->is_fire;
    packet.x = 3;
    packet.y = msg->position.y;
    packet.z = msg->position.z;
    packet.v_yaw = msg->v_yaw;

    packet.pitch = msg->pitch;

    packet.yaw = msg->yaw;

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

    //打印 data 结构体中的 xyz 和 yaw 值
    std::cout << "xyz: (" << packet.x << ", " << packet.y << ", " << packet.z << ")" << std::endl;
    std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;
    RCLCPP_INFO(get_logger(), "[Send] aim_x %f!", packet.x);
    RCLCPP_INFO(get_logger(), "[Send] aim_y %f!", packet.y);
    RCLCPP_INFO(get_logger(), "[Send] aim_z %f!", packet.z);

    RCLCPP_INFO(get_logger(), "-------------------------------------------------------------");
    RCLCPP_INFO(get_logger(), "[Send] pitch %f!", packet.pitch);
    RCLCPP_INFO(get_logger(), "[Send] yaw %f!", packet.yaw);
    RCLCPP_INFO(get_logger(), "-------------------------------------------------------------");


    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);//通过串口发送 data

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;//计算从接收到消息到发送之间的延迟时间，并将其转换为毫秒
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");//使用 RCLCPP_DEBUG_STREAM 记录延迟时间
    latency_pub_->publish(latency);//发布延迟时间到 latency_pub_

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());//捕获所有异常，记录错误日志，并尝试重新打开串口
    reopenPort();
  }
}

void RMSerialDriver::getParams()//从ROS参数服务器中获取串口配置参数，包括设备名称、波特率、流控制、奇偶校验和停止位，并将这些参数封装到一个SerialPortConfig对象中
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {//尝试从参数服务器获取设备名称，如果失败则记录错误日志并抛出异常
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {//从参数服务器获取流控制参数，并根据字符串值设置相应的枚举值。如果参数无效则抛出异常
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {//从参数服务器获取奇偶校验参数，并根据字符串值设置相应的枚举值
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {//从参数服务器获取停止位参数，并根据字符串值设置相应的枚举值
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ = //将获取到的参数封装到一个SerialPortConfig对象中，并赋值给成员变量device_config_
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)//检查参数服务是否准备好
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (//如果当前没有正在进行的参数设置请求或已有请求已完成，则发送新的参数设置请求
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()//检查重置跟踪器服务是否准备好
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

float RMSerialDriver::pitch_trans(float originAngle){//将输入的角度转换为正值（0到2π之间）

    if (originAngle < 0) {//如果角度为负值，则加上2π
      originAngle = originAngle +2* M_PI;
    }
    else
      originAngle = originAngle;
    return originAngle;
    // return originAngle;
}


float RMSerialDriver::pitch_re_trans(float originAngle){//将输入的角度转换为负值（-π到π之间）

  if (originAngle <= M_PI) {//如果角度大于π，则减去2π
    originAngle = originAngle;
  }
  else
    originAngle = originAngle -2*M_PI;


      return originAngle;
  // return originAngle-M_PI;
}

float RMSerialDriver::yaw_trans(float originAngle){//将输入的角度转换为正值（0到2π之间）

    if (originAngle <=0) {
      originAngle = abs(originAngle);//取绝对值
    }
    else
      originAngle = 2 * M_PI - originAngle;
    return originAngle;


}
float RMSerialDriver::yaw_re_trans(float originAngle){//将输入的角度转换为负值

    if (originAngle <= M_PI) {
      originAngle = -originAngle;
    }
    else
      originAngle = 2*M_PI  -originAngle;
    return originAngle;


}


}  // namespace rm_serial_driver


/*通过使用rclcpp_components库提供的宏，将rm_serial_driver::RMSerialDriver节点注册到类加载器中，从而使其在运行时可以被动态发现和加载。
这对于构建模块化和可扩展的ROS 2应用程序非常有用*/
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.将组件注册到类加载器中。
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.当其库被加载到一个运行的进程中时，允许该组件被发现
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
