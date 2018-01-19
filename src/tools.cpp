#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  if (estimations.size()!=ground_truth.size())
  {
    std::cout << "Size mismatch!" << std::endl;
  } else if (estimations.size()==0) {
    return VectorXd::Zero(0);
  } else {
    VectorXd rmse = VectorXd::Zero(estimations[0].size());
    unsigned int N = estimations.size();
    for (unsigned int i=0; i<N; i++) {
        VectorXd diff = estimations[i] - ground_truth[i];
        VectorXd diff2 = diff.array() * diff.array();
        rmse += diff2;
    }
    rmse /= N;
    rmse = rmse.array().sqrt();
    return rmse;
  }
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  
  // x = [x, y, vx, vy]'
  // z = [ro, theta, ro_dot]'
  // ro = sqrt(x^2+y^2)
  // theta = atan2(y, x)
  
  // Compute Jacobian Hj
  double x = x_state(0);
  double y = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  double ro = std::hypot(x, y);
  double ro_dot = (x*vx+y*vy)/ro;
  Eigen::MatrixXd Hj(3, 4);
  Hj.row(0) << x/ro, y/ro, 0, 0;
  double ro2 = x*x+y*y;
  double ro3 = std::pow(ro, 3);
  Hj.row(1) << -y/ro2, x/ro2, 0, 0;
  Hj.row(2) << y*(vx*y-vy*x)/ro3, x*(vy*x-vx*y)/ro3, x/ro, y/ro;
  return Hj;
}
