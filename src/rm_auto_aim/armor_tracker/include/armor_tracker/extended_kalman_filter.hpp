#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  explicit ExtendedKalmanFilter(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VecMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

  // Set the initial state 滤波器初始状态
  void setState(const Eigen::VectorXd & x0);

  // Compute a predicted state 根据系统模型预测下一个状态
  Eigen::MatrixXd predict();

  // Update the estimated state based on measurement 根据观测值更新状态
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
  // Process nonlinear vector function 系统的非线性状态转移函数，它接受当前状态向量作为输入，返回预测的下一个状态向量。
  VecVecFunc f;
  // Observation nonlinear vector function 观测的非线性函数，它将系统的状态向量映射到观测空间，接受状态向量作为输入，返回对应的观测向量
  VecVecFunc h;
  // Jacobian of f() jacobian_f 是一个函数对象，用于计算系统状态转移函数 f 的雅可比矩阵。F 是存储计算得到的系统状态转移函数的雅可比矩阵的矩阵对象。
  VecMatFunc jacobian_f;
  Eigen::MatrixXd F;
  // Jacobian of h()  jacobian_h 是一个函数对象，用于计算观测函数 h 的雅可比矩阵。H 是存储计算得到的观测函数的雅可比矩阵的矩阵对象。
  VecMatFunc jacobian_h;
  Eigen::MatrixXd H;
  // Process noise covariance matrix update_Q 是一个函数对象，用于更新过程噪声协方差矩阵。Q 是存储当前过程噪声协方差矩阵的矩阵对象。
  // 过程噪声协方差矩阵描述了系统状态转移过程中的不确定性，它是一个 NxN 的矩阵，其中 N 是状态向量的维度。
  VecMatFunc update_Q;
  Eigen::MatrixXd Q;
  // Measurement noise covariance matrix update_R 是一个函数对象，用于更新测量噪声协方差矩阵 R 是存储当前测量噪声协方差矩阵的矩阵对象。
  VecMatFunc update_R;
  Eigen::MatrixXd R;

  // Priori error estimate covariance matrix        P_pri 是先验误差估计协方差矩阵，用于在预测步骤中表示状态估计的不确定性。
  Eigen::MatrixXd P_pri;
  // Posteriori error estimate covariance matrix    P_post是后验误差估计协方差矩阵，用于在更新步骤后表示状态估计的不确定性。
  Eigen::MatrixXd P_post;

  // Kalman gain K 是卡尔曼增益矩阵，用于在更新步骤中权衡预测值和测量值的权重。
  Eigen::MatrixXd K;

  // System dimensions n 表示系统的状态维度，即状态向量的长度。
  int n;

  // N-size identity
  Eigen::MatrixXd I;

  // Priori state x_pri 是先验状态向量，是预测步骤得到的状态估计。
  Eigen::VectorXd x_pri;
  // Posteriori state  x_post 是后验状态向量，是更新步骤后得到的状态估计。
  Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
