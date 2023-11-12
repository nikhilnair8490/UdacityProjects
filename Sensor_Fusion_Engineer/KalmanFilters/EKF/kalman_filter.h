#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
 public:

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Predict Predicts the state and the state covariance
   *   using the process model
   */
  void Predict();

  /**
   * Updates the state and
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);

  // New update for EKF
  void UpdateEKF(const VectorXd &z);

  // Calculate jacobian
  MatrixXd CalculateJacobian();
  
  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // state transistion matrix
  MatrixXd F_;

  // process covariance matrix
  MatrixXd Q_;

  // measurement matrix
  MatrixXd H_;
  MatrixXd Hj_; // Jacobian for RADAR

  // measurement covariance matrix
  MatrixXd R_; // laser Covariance
  MatrixXd Rr_; // Radar Covariance

};

#endif  // KALMAN_FILTER_H_