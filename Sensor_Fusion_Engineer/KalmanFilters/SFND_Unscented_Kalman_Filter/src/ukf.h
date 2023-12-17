#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF
{
public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * @brief Generate augmented sigma points
   *
   * @param Xsig_aug Augmented Sigma Points
   */
  void AugmentedSigmaPoints(Eigen::MatrixXd &Xsig_aug);

  /**
   * @brief Predict sigma points
   *
   * @param Xsig_aug  Augmented sigma points
   * @param Xsig_pred Predicted sigma points
   * @param delta_t   Time between sensor measurements
   */
  void SigmaPointPrediction(const Eigen::MatrixXd &Xsig_aug, Eigen::MatrixXd &Xsig_pred_,
                            double delta_t);

  /**
   * @brief Predict state mean and process covariance
   *
   * @param Xsig_pred Predicted sigma points
   */
  void PredictMeanAndCovariance(const Eigen::MatrixXd &Xsig_pred_);

  /**
   * @brief Predict radar states in measurement space
   * 
   * @param z_pred  Radar predicted state vector in measurement space
   * @param S       Radar measurement covariance matrix
   * @param Zsig    Radar sigma points matrix in measurement space
   */
  void PredictRadarMeasurement(Eigen::VectorXd &z_pred, Eigen::MatrixXd &S, Eigen::MatrixXd &Zsig);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z_radar_;

  //Lidar can measure px, py;
  int n_z_lidar_;

  // NIS value for radar
  float nisRadar_ = 0.0;

  // NIS value for lidar
  float nisLidar_ = 0.0;

};

#endif // UKF_H