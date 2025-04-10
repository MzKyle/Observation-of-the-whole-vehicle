#ifndef BUFF_TRACKER__TRACKER_NODE_HPP_
#define BUFF_TRACKER__TRACKER_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <buff_interfaces/msg/blade.hpp>
#include <buff_interfaces/msg/blade_array.hpp>
#include <buff_interfaces/msg/rune.hpp>
#include <buff_interfaces/msg/rune_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "buff_tracker/extended_kalman_filter.hpp"
#include "buff_tracker/gauss_newton_solver.hpp"
#include "buff_tracker/tracker.hpp"

namespace rm_buff
{

using tf2_filter = tf2_ros::MessageFilter<buff_interfaces::msg::BladeArray>;

class BuffTrackerNode : public rclcpp::Node
{
public:
  explicit BuffTrackerNode(const rclcpp::NodeOptions & options);

private:
  void bladesCallback(const buff_interfaces::msg::BladeArray::SharedPtr blades_msg);
  double dt_;//时间间隔，用于计算时间差

  rclcpp::Time last_time_;//上次接收数据的时间

  double lost_time_threshold_;//丢失时间阈值，用于判断是否丢失目标

  std::unique_ptr<Tracker> tracker_;

  // param
  //噪声
  double s2qxyz_;
  double s2qtheta_;
  double s2qr_;
  double r_blade_;
  double r_center_;

  int max_iter_;//表示最大迭代次数。在使用迭代算法（如高斯牛顿法）求解问题时，为了避免无限循环，会设置一个最大迭代次数，当达到这个次数时，算法停止迭代。
  double min_step_;//表示最小步长。在迭代过程中，如果每次迭代的步长小于这个值，说明算法已经收敛到一个较优的解，此时可以停止迭代。
  int obs_max_size_;//表示观测值的最大数量。在跟踪过程中，可能会存储一定数量的观测值用于后续的处理和分析，这个参数限制了观测值的最大存储数量

  //分别表示加速度的最大值和最小值。在跟踪目标的运动过程中，为了避免不合理的加速度值，会设置一个合理的范围
  double max_a_;
  double min_a_;

  //分别表示角速度的最大值和最小值。同样，为了避免不合理的角速度值，会设置一个合理的范围。
  double max_w_;
  double min_w_;

  //协方差噪声
  double s2q_a_;
  double s2q_w_;
  double s2q_c_;
  double r_a_;
  double r_w_;
  double r_c_;

  //  task subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_; //任务订阅者，用于订阅任务模式消息
  std::string task_mode_;//当前的任务模式
  void taskCallback(const std_msgs::msg::String::SharedPtr task_msg);

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<buff_interfaces::msg::BladeArray> blades_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Publisher
  rclcpp::Publisher<buff_interfaces::msg::Rune>::SharedPtr rune_publisher_;
  rclcpp::Publisher<buff_interfaces::msg::RuneInfo>::SharedPtr rune_info_publisher_;

  // visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr blade_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr center_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr measure_marker_pub_;
  visualization_msgs::msg::Marker blade_marker_;
  visualization_msgs::msg::Marker center_marker_;
  visualization_msgs::msg::Marker measure_marker_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pnp_result_pub_;
};
}  // namespace rm_buff

#endif  // BUFF_TRACKER__TRACKER_NODE_HPP_