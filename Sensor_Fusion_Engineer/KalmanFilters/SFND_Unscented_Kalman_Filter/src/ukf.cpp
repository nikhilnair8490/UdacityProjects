#include "ukf.h"
#include "Eigen/Dense"

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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  // Intialize the process covariance matrix
  P_ << 1, 0, 0, 0, 0,
      0, 1, 0, 0, 0,
      0, 0, 1, 0, 0,
      0, 0, 0, 1, 0,
      0, 0, 0, 0, 1;

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
  Eigen::MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  UKF::SigmaPointPrediction(Xsig_aug, Xsig_pred, delta_t);

  /* Predict state mean and process covariance*/
  UKF::PredictMeanAndCovariance(Xsig_pred);
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
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

void UKF::SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, Eigen::MatrixXd &Xsig_pred,
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

    Xsig_pred.col(i) = Xsig_augVec.head(n_x_) + stateVec + noiseVec;
  }
}

void UKF::PredictMeanAndCovariance(const Eigen::MatrixXd &Xsig_pred)
{

  // predict state mean

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ += weights_(i) * (Xsig_pred.col(i));
  }

  // predict state covariance matrix

  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd errorVec = Xsig_pred.col(i) - x_;

    P_ += weights_(i) * errorVec * errorVec.transpose();
  }
}