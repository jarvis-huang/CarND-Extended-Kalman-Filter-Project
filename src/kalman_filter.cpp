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
  Eigen::VectorXd y = z - H_ * x_; // innovation
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ += K * y;
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  // x = [x, y, vx, vy]'
  // z = [ro, theta, ro_dot]'
  // ro = sqrt(x^2+y^2)
  // theta = atan2(y, x)
  
  double _x = x_(0);
  double _y = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double ro = std::hypot(_x, _y);
  double theta = std::atan2(_y, _x);
  while (theta<-M_PI) theta+=2*M_PI;
  while (theta>M_PI) theta-=2*M_PI;
  double ro_dot = (_x*vx+_y*vy)/ro;

  Eigen::VectorXd z_(z.size());
  z_ << ro, theta, ro_dot;
  Eigen::VectorXd y = z - z_; // innovation
  // Normalize angle difference
  while (y(1)<-M_PI) y(1)+=2*M_PI;
  while (y(1)>M_PI) y(1)-=2*M_PI;  
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ += K * y;
  P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K*H_) * P_;
  
}
