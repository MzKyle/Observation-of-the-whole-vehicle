#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "armor_executor/SolveTrajectory.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_interfaces/msg/velocity.hpp"
#include "auto_aim_interfaces/msg/send.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  void velocityCallback(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg); //接收到的速度消息的回调函数。

  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);  //接收到的装甲板消息的回调函数。

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);  //发布可视化标记的函数。

  // Maximum allowable armor distance in the XOY plane     XOY平面上允许的最大装甲板距离。
  double max_armor_distance_;

  // The time when the last message was received    last_time_ 和 dt_：分别存储上一次接收到消息的时间和时间间隔。
  rclcpp::Time last_time_;
  double dt_;

  // Armor tracker
  double s2qxyz_max_, s2qxyz_min_, s2qyaw_max_, s2qyaw_min_, s2qr_; //界定阈值
  double r_xyz_factor, r_yaw;
  double lost_time_thres_; //丢失目标的时间阈值
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<SolveTrajectory> gaf_solver;

  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // Change target service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_target_srv_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Velocity>::SharedPtr velocity_sub_; // 订阅速度消息

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr send_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
