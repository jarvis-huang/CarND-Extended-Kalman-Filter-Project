#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  F_ = MatrixXd(4, 4);
  Q_ = MatrixXd(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process andCalculateJacobian measurement noises
  */
          
  noise_ax = 9;
  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      Eigen::VectorXd z = measurement_pack.raw_measurements_;
      double ro = z(0);
      double theta = z(1);
      double ro_dot = z(2);
      Eigen::VectorXd x_init(4);
      x_init << ro*cos(theta), ro*sin(theta), ro_dot*cos(theta), ro_dot*sin(theta);
      Eigen::MatrixXd dummy = MatrixXd::Identity(4, 4);
      Eigen::MatrixXd dummy2 = MatrixXd::Zero(3, 4);
      Eigen::MatrixXd dummy3 = MatrixXd::Zero(3, 3);
      // Init(x, P, F, H, R, Q)
      ekf_.Init(x_init, dummy, dummy, dummy2, R_radar_, dummy);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      Eigen::VectorXd z = measurement_pack.raw_measurements_;
      double x = z(0);
      double y = z(1);
      Eigen::VectorXd x_init(4);
      x_init << x, y, 0, 0;
      Eigen::MatrixXd dummy = MatrixXd::Identity(4, 4);
      H_laser_ << 1, 0, 0, 0,
                  0, 1, 0, 0;
      // Init(x, P, F, H, R, Q)
      ekf_.Init(x_init, dummy, dummy, H_laser_, R_laser_, dummy);
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // Timestamp is measured in us
  double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  double dt2 = dt*dt;
  double dt3 = dt2*dt;
  double dt4 = dt2*dt2;
  double var_ax = noise_ax*noise_ax;
  double var_ay = noise_ay*noise_ay;
  // motion matrix
  F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
  // process noise
  Q_ << dt4/4*var_ax, 0, dt3/2*var_ax, 0,
        0, dt4/4*var_ay, 0, dt3/2*var_ay,
        dt3/2*var_ax, 0, dt2*var_ax, 0,
        0, dt3/2*var_ay, 0, dt2*var_ay;
  ekf_.F_ = F_;
  ekf_.Q_ = Q_;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  previous_timestamp_ = measurement_pack.timestamp_;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  
  
}
