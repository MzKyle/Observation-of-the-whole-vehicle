#ifndef BUFF_TRACKER__TRACKER_HPP_
#define BUFF_TRACKER__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

// STD
#include <angles/angles.h>

#include <buff_interfaces/msg/blade.hpp>
#include <buff_interfaces/msg/blade_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <string>

#include "buff_tracker/extended_kalman_filter.hpp"
#include "buff_tracker/gauss_newton_solver.hpp"

#define PI 3.1415926
#define BLADE_R_OFFSET 700.0  //扇叶半径偏移量
// #define OMEGA 1.0 / 3 * PI
#define OMEGA 0.0
namespace rm_buff
{
class Tracker
{
public:
  Tracker(double max_match_theta, double max_match_center_xoy);

  void init(const buff_interfaces::msg::BladeArray::SharedPtr & blades_msg);//根据接收到的扇叶消息初始化跟踪器

  void update(const buff_interfaces::msg::BladeArray::SharedPtr & blades_msg);//根据接收到的扇叶消息更新跟踪器

  void solve(const rclcpp::Time & time);//在指定时间点进行求解

  //三个求解器
  ExtendedKalmanFilter ekf;

  GaussNewtonSolver gns;

  ExtendedKalmanFilter ekf_gns;

  double a_start, w_start, c_start;
  double min_first_solve_time;  //最小首次求解时间

  //阈值参数
  int tracking_threshold;//踪阈值，用于判断跟踪器是否进入跟踪状态。当满足一定条件（如连续检测到目标的次数）达到该阈值时，跟踪器进入跟踪状态。
  int lost_threshold;//丢失阈值，用于判断跟踪器是否丢失目标。当连续未检测到目标的次数达到该阈值时，跟踪器认为目标丢失。
  int timeout_threshold;//超时阈值，单位为毫秒（ms），子弹飞行时间不满足毫秒。当跟踪过程中的某些条件（如目标运动速度、距离等）导致子弹飞行时间不足时，跟踪器可能进入超时状态。

  double blade_z_ground;//扇叶距离地面的高度
  double robot_z_ground;//机器人距离地面的高度
  double distance;
  double max_distance_diff;//最大距离差值，用于判断目标的距离变化是否在合理范围内
  double center_xoy_diff;//中心点在 x 和 y 平面上的差值，用于判断目标的位置变化是否在合理范围内

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TIMEOUT,
    TEMP_LOST,
  } tracker_state;

  enum SolverStatus {
    WAITING,//求解器正在等待足够的观测数据
    NOT_ENOUGH_OBS,//观测数据不足，无法进行有效求解
    VALID,//结果有效
    INVALID,//结果无效
  } solver_status;

  // blade info
  // first TRACKING blade set to blade_id = 0
  // diff with blade_0: phi
  // blade_id: 0 <== phi = 0/5 * pi
  //           1 <== phi = 2/5 * pi
  //           2 <== phi = 4/5 * pi
  //           3 <== phi = 6/5 * pi
  //           4 <== phi = 8/5 * pi
  int blade_id;

  struct blade_transform
  {
    geometry_msgs::msg::Point blade_position;
    geometry_msgs::msg::Point center_position;
    double theta;
  };

  blade_transform tracked_blade;

  Eigen::VectorXd measurement; //测量值

  Eigen::VectorXd target_state; //目标状态

  Eigen::VectorXd spd_state;  //速度状态

  rclcpp::Time obs_start_time; //是否到达时间阈值

  void getTrackerPosition(blade_transform & blade);

private:
  void initEKF(const blade_transform & blade);

  blade_transform bladeTransform(const buff_interfaces::msg::Blade & blade);

  bool handleBladeJump(double theta_diff);//处理扇叶角度跳跃的情况

  geometry_msgs::msg::Point rotateBlade(const blade_transform blade, int idx);

  void calculateMeasurementFromPrediction(blade_transform & blade, const Eigen::VectorXd & state);

  double max_match_theta_;
  double max_match_center_xoy_;

  int detect_count_;
  int lost_count_;
  int timeout_count_;  // ms

  double last_theta_;
};
}  // namespace rm_buff

#endif  // BUFF_TRACKER__TRACKER_HPP_