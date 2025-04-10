#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

enum class ArmorsNum { NORMAL_4 = 4,OUTPOST_3 = 3 }; //normal四个装甲板，2025年wheel leg也改为4个装甲板，前哨站三个装甲板

class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff_);  //两帧间目标可匹配的最大距离，可匹配的最大yaw差

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg); //初始化跟踪器，跟踪目标装甲板

  void update(const Armors::SharedPtr & armors_msg); //更新跟踪器，更新当前瞄准的装甲板

  void adaptAngularVelocity(const double & duration);  //调整角速度

  ExtendedKalmanFilter ekf;

  int tracking_thres;
  int lost_thres;
  int change_thres;


  //四个状态分别是：1.detecting 2.tracking 3.temp_lost 4.lost 5.change_target
  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    CHANGE_TARGET,
  } tracker_state;

  std::string tracked_id;  //当前跟踪的装甲板的ID
  std::string last_tracked_id;  //上一次跟踪的装甲板的ID
  Armor tracked_armor;  //当前跟踪的装甲板的信息
  ArmorsNum tracked_armors_num;  //当前跟踪的装甲板的数量

  double info_position_diff;  //位置差异
  double info_yaw_diff;  //yaw差异

  Eigen::VectorXd measurement;  //测量值向量

  Eigen::VectorXd target_state;  //目标状态向量

  // To store another pair of armors message
  double dz, another_r;

private:
  void initEKF(const Armor & a);  //对一个装甲板初始化EKF

  void initChange(const Armor & armor_msg); //初始化切换目标操作

  void updateArmorsNum(const Armor & a);  //更新当前跟踪目标的装甲板数量

  void handleArmorJump(const Armor & a);  //处理装甲板跳跃状况

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);  //四元数转yaw角

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);  //从状态向量中获取装甲板位置

  double max_match_distance_;  //两帧间目标可匹配的最大距离

  double max_match_yaw_diff_;  //两帧间目标可匹配的最大yaw差

  int detect_count_;  //检测计数器，用于记录目标检测的次数
  int lost_count_;    //丢失目标计数器，用于记录目标丢失的次数
  int change_count_; //切换目标计数器，用于记录更换目标的次数

  double last_yaw_;  //上一次的yaw角
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
