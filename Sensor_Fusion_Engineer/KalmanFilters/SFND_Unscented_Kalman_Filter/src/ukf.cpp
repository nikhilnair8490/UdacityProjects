#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.70;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.60;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  // Initialize UKF class internal variables
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  n_z_radar_ = 3;
  n_z_lidar_ = 2;
  time_us_ = 0.0;

  // Intialize the process covariance matrix
  P_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 100, 0, 0,
      0, 0, 0, 100, 0,
      0, 0, 0, 0, 100;

  // Intialize predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // Initialize weights for sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  for (int i = 1; i < 2 * n_aug_ + 1; ++i)
  {
    weights_(i) = 1.0 / (2.0 * (lambda_ + n_aug_));
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_)
  {
    std::cout << "Kalman Filter Initialization " << std::endl;

    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      // set the state with the initial location and zero velocity
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1],
          0,
          0,
          0;
    }
    else // Radar
    {
      // convert radar measurements from polar to cartesian coordinates
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];

      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);

      x_ << px, py, v, 0, 0;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // 1. Call the predict() function
  UKF::Prediction(dt);

  // 2. Call the update() function
  //      with the most recent raw measurements_
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UKF::UpdateLidar(meas_package);
  }
  else // RADAR so use UKF
  {
    UKF::UpdateRadar(meas_package);
  }

  std::cout << "x_= \n"
            << x_ << std::endl;
  std::cout << "P_= \n"
            << P_ << std::endl;
}

void UKF::Prediction(double delta_t)
{
  /**
   * Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */

  /*Generate Augmented Sigma Points*/
  Eigen::MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  UKF::AugmentedSigmaPoints(Xsig_aug);

  /* Predict Sigma Points */
  UKF::SigmaPointPrediction(Xsig_aug, Xsig_pred_, delta_t);

  /* Predict state mean and process covariance*/
  UKF::PredictMeanAndCovariance(Xsig_pred_);
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * Use lidar data to update the belief
   * about the object's position.
   */

  // Update state using lidar measurement
  // Since Lidar measurement transformation is linear we use the standard KF

  // create vector for incoming lidar measurement
  VectorXd z = VectorXd(n_z_lidar_);
  z = meas_package.raw_measurements_;

  // create measurement matrix for lidar (Lidr has only two px,py measurements)
  MatrixXd H_ = MatrixXd(n_z_lidar_, n_x_);
  H_.fill(0.0);
  H_(0, 0) = 1;
  H_(1, 1) = 1;

  // measurement noise matrix R
  MatrixXd R = MatrixXd(n_z_lidar_, n_z_lidar_);
  R.fill(0.0);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  // Calculate NIS for lidar measurement

  VectorXd zDiffNIS = VectorXd(n_z_lidar_);
  zDiffNIS = z - z_pred;
  nisLidar_ = zDiffNIS.transpose() * S.inverse() * zDiffNIS;
  std::cout << "NIS Lidar:" << nisLidar_ << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * Use radar data to update the belief
   * about the object's position.
   */

  // Since the radar measurement transformation is non-linear, we use UKF method

  // Create radar predicted measurement vector
  VectorXd z_pred = VectorXd(n_z_radar_);
  // Create radar covariance matrix
  MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  // Predict radar measurement
  UKF::PredictRadarMeasurement(z_pred, S, Zsig);

  // Update state using the radar measurement

  // create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_radar_);
  z = meas_package.raw_measurements_;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  { // 2n+1 simga points
    // residual
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    while (x_diff(3) > M_PI)
      x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI)
      x_diff(3) += 2. * M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z_radar_);
  K.fill(0.0);

  K = Tc * S.inverse();

  // update state mean and covariance matrix

  x_ = x_ + K * (z - z_pred);
  P_ = P_ - K * S * K.transpose();

  // Calculate NIS for radar measurement

  VectorXd zDiffNIS = VectorXd(n_z_radar_);
  zDiffNIS = z - z_pred;
  nisRadar_ = zDiffNIS.transpose() * S.inverse() * zDiffNIS;
  std::cout << "NIS Radar:" << nisRadar_ << std::endl;
}

// Additional Functions

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd &Xsig_aug)
{

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // Process noise covariance matrix
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a_ * std_a_, 0,
      0, std_yawdd_ * std_yawdd_;

  // create augmented mean state
  x_aug.head(5) = x_;

  // create augmented covariance matrix
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  // create square root matrix
  MatrixXd A_aug = P_aug.llt().matrixL();

  // create augmented sigma points

  // set first column of sigma point matrix
  Xsig_aug.col(0) = x_aug;

  // set remaining sigma points
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(i);
  }
}

void UKF::SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, Eigen::MatrixXd &Xsig_pred_,
                               double delta_t)
{
  // predict sigma points

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {

    VectorXd Xsig_augVec = Xsig_aug.col(i);
    double v_k = Xsig_aug(2, i);
    double psi_k = Xsig_aug(3, i);
    double psiDot_k = Xsig_aug(4, i);
    double accNoise_k = Xsig_aug(5, i);
    double yawRateNoise_k = Xsig_aug(6, i);

    VectorXd stateVec(n_x_);
    VectorXd noiseVec(n_x_);

    if (psiDot_k != 0.0)
    {
      stateVec << (v_k / psiDot_k) * (sin(psi_k + psiDot_k * delta_t) - sin(psi_k)),
          (v_k / psiDot_k) * (-cos(psi_k + psiDot_k * delta_t) + cos(psi_k)),
          0,
          psiDot_k * delta_t,
          0;

      noiseVec << 0.5 * (delta_t * delta_t) * cos(psi_k) * accNoise_k,
          0.5 * (delta_t * delta_t) * sin(psi_k) * accNoise_k,
          delta_t * accNoise_k,
          0.5 * (delta_t * delta_t) * yawRateNoise_k,
          delta_t * yawRateNoise_k;
    }
    else
    {
      stateVec << v_k * cos(psi_k) * delta_t,
          v_k * sin(psi_k) * delta_t,
          0,
          0,
          0;

      noiseVec << 0.5 * (delta_t * delta_t) * cos(psi_k) * accNoise_k,
          0.5 * (delta_t * delta_t) * sin(psi_k) * accNoise_k,
          delta_t * accNoise_k,
          0.5 * (delta_t * delta_t) * yawRateNoise_k,
          delta_t * yawRateNoise_k;
    }

    Xsig_pred_.col(i) = Xsig_augVec.head(n_x_) + stateVec + noiseVec;
  }
}

void UKF::PredictMeanAndCovariance(const Eigen::MatrixXd &Xsig_pred_)
{

  // predict state mean

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * (Xsig_pred_.col(i));
  }

  // predict state covariance matrix

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd errorVec = Xsig_pred_.col(i) - x_;

    P_ += weights_(i) * errorVec * errorVec.transpose();
  }
}

void UKF::PredictRadarMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig)
{

  // measurement noise matrix R
  MatrixXd R = MatrixXd(n_z_radar_, n_z_radar_);
  R.fill(0.0);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd currSigPt = Xsig_pred_.col(i);

    double p_x = currSigPt(0);
    double p_y = currSigPt(1);
    double v = currSigPt(2);
    double psi = currSigPt(3);
    double psiDot = currSigPt(4);

    double rho = sqrt(p_x * p_x + p_y * p_y);
    double phi = std::atan2(p_y, p_x);
    double rhoDot = (p_x * cos(psi) * v + p_y * sin(psi) * v) / rho;

    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rhoDot;
  }

  // calculate mean predicted measurement

  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  { // iterate over sigma points
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // predicted measurement covariance matrix
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  { // iterate over sigma points
    // state difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
      z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
      z_diff(1) += 2. * M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  S = S + R;
}