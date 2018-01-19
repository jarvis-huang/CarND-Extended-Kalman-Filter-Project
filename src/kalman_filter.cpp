#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  std::cout << "C" << std::endl;
  Eigen::VectorXd y = z - H_ * x_; // innovation
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ += K * y;
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K*H_) * P_;
  std::cout << "D" << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  // x = [x, y, vx, vy]'
  // z = [ro, theta, ro_dot]'
  // ro = sqrt(x^2+y^2)
  // theta = atan2(y, x)
  
  std::cout << "A" << std::endl;
  
  // Compute Jacobian Hj
  double _x = x_(0);
  double _y = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double ro = std::hypot(_x, _y);
  double theta = std::atan2(_y, _x);
  while (theta<-M_PI) theta+=2*M_PI;
  while (theta>M_PI) theta-=2*M_PI;
  double ro_dot = (_x*vx+_y*vy)/ro;
  Eigen::MatrixXd Hj(3, 4);
  Hj.row(0) << _x/ro, _y/ro, 0, 0;
  double ro2 = _x*_x+_y*_y;
  double ro3= std::pow(ro, 3);
  Hj.row(1) << -_y/ro2, _x/ro2, 0, 0;
  Hj.row(2) << _y*(vx*_y-vy*_x)/ro3, _x*(vy*_x-vx*_y)/ro3, _x/ro, _y/ro;
  
  Eigen::VectorXd z_(z.size());
  z_ << ro, theta, ro_dot;
  Eigen::VectorXd y = z - z_; // innovation
  Eigen::MatrixXd S = Hj * P_ * Hj.transpose() + R_;
  Eigen::MatrixXd K = P_ * Hj.transpose() * S.inverse();
  x_ += K * y;
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K*Hj) * P_;
  std::cout << "B" << std::endl;
}
