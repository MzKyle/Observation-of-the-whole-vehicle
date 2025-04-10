#include "armor_tracker/extended_kalman_filter.hpp"

namespace rm_auto_aim
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_post(P0),
  n(P0.rows()), // 状态向量的维度
  I(Eigen::MatrixXd::Identity(n, n)), // 单位矩阵
  x_pri(n),  //先验状态向量，是预测步骤得到的状态估计
  x_post(n)  //后验状态向量，是更新步骤后得到的状态估计
{
}

/*
f：系统状态转移函数，接受当前状态向量，返回预测的下一个状态向量。
h：观测函数，将系统状态向量映射到观测空间。
j_f：系统状态转移函数的雅可比矩阵计算函数。
j_h：观测函数的雅可比矩阵计算函数。
u_q：过程噪声协方差矩阵更新函数。
u_r：测量噪声协方差矩阵更新函数。
P0：初始的后验误差协方差矩阵。
*/


void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post), Q = update_Q(x_post);

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;  //返回预测的状态向量
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = jacobian_h(x_pri), R = update_R(z);

  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;

  return x_post;  //返回更新后的状态向量
}

}  // namespace rm_auto_aim
