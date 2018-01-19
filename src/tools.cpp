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
}
