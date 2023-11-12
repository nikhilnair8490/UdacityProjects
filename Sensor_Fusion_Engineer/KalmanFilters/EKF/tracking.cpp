#include "tracking.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

Tracking::Tracking() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // create a 4D state vector, we don't know yet the values of the x state
  kf_.x_ = VectorXd(4);

  // state covariance matrix P
  kf_.P_ = MatrixXd(4, 4);
  kf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


  // measurement covariance
  kf_.R_ = MatrixXd(2, 2);
  kf_.R_ << 0.0225, 0,
            0, 0.0225;
  kf_.Rr_ = MatrixXd(3, 3);
  kf_.Rr_ << 0.0225, 0, 0,
            0, 0.0225, 0,
            0, 0, 0.0225;

  // measurement matrix
  kf_.H_ = MatrixXd(2, 4);
  kf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  kf_.Hj_ = MatrixXd(3,4);
  kf_.Hj_ << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;

  // the initial transition matrix F_
  kf_.F_ = MatrixXd(4, 4);
  kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // set the acceleration noise components
  noise_ax = 5;
  noise_ay = 5;
}

Tracking::~Tracking() {

}

// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    //cout << "Kalman Filter Initialization " << endl;

    // set the state with the initial location and zero velocity
    kf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // 1. Modify the F matrix so that the time is integrated
  kf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
  // 2. Set the process covariance matrix Q
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 =  dt3*dt;
  kf_.Q_ = MatrixXd(4, 4);
  kf_.Q_ << dt4*noise_ax/4.0, 0, dt3*noise_ax/2.0, 0,
            0, dt4*noise_ay/4.0, 0, dt3*noise_ay/2.0,
            dt3*noise_ax/2.0, 0, dt2*noise_ax, 0,
            0, dt3*noise_ay/2.0, 0, dt2*noise_ay;

  // 3. Call the Kalman Filter predict() function
  kf_.Predict();

  // 4. Call the Kalman Filter update() function
  //      with the most recent raw measurements_
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
  {
    kf_.Update(measurement_pack.raw_measurements_);
  }
  else // RADAR so use EKF
  {
    kf_.Hj_ = kf_.CalculateJacobian();
    kf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  
  cout << "x_= " << kf_.x_ << endl;
  cout << "P_= " << kf_.P_ << endl;
}