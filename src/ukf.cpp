#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.4;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  // set state dimension
  n_x_ = 5;

  // set sensor state dimensions
  n_radar_ = 3;
  n_laser_ = 2;

  // set augmented dimension
  n_aug_ = 7;

  // set lambda
  lambda_ = 3 - n_aug_;
  
  // initialize prediction state matrix
  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);

  // initialize sigma point weights
  weights_ = VectorXd(2*n_aug_+1);

  // normalized innovation data parameters
  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;

  // set initialized state to false
  is_initialized_ = false;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  /*****************************************************************************
   *  Filter Measurements
   ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) 
    return;
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_)
    return;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // set first measurement
    x_ << 0, 0, 0, 0, 0;
    // set covariance matrix
    P_ << std_laspx_, 0, 0, 0, 0,
          0, std_laspy_, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];

      // Coordinates convertion from polar to cartesian
      x_(0) = (rho * cos(phi) >= 0.0001) ? (rho * cos(phi)) : 0.0001;
      x_(1) = (rho * sin(phi) >= 0.0001) ? (rho * sin(phi)) : 0.0001;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_(0);  // position x
      x_(1) = meas_package.raw_measurements_(1);  // position y
 
    }

    //set weights
    weights_(0) = lambda_/(lambda_+n_aug_);
    for (int i=1; i<2*n_aug_+1; i++)
      weights_(i) = 0.5/(n_aug_+lambda_);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return; 
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  Prediction(delta_t); 

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
    //std::cout << NIS_radar_ << std::endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
    //std::cout << NIS_laser_ << std::endl;
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  /*****************************************************************************
   *  Generate Sigma Points (Lesson 8.13)
   ****************************************************************************/
  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;

  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_(5, 5) = std_a_*std_a_;
  P_aug_(6, 6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented mean state
  Xsig_aug_.col(0) = x_aug_;

  //create augmented signma points
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i + 1)          = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /*****************************************************************************
   *  Predict Sigma Points (Lesson 8.19)
   ****************************************************************************/
  for (int i = 0; i < 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x      = Xsig_aug_(0,i);
    double p_y      = Xsig_aug_(1,i);
    double v        = Xsig_aug_(2,i);
    double yaw      = Xsig_aug_(3,i);
    double yawd     = Xsig_aug_(4,i);
    double nu_a     = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /*****************************************************************************
   *  Predict Mean and Covariance (Lesson 8.22)
   ****************************************************************************/
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  /*****************************************************************************
   *  Measurement Prediction (Lesson 8.25)
   ****************************************************************************/
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_laser_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig_(0,i) = p_x;       // position x
    Zsig_(1,i) = p_y;       // position y
  }

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_laser_);
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++)
      z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);

  //innovation covariance matrix S
  MatrixXd S_ = MatrixXd(n_laser_,n_laser_);
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff_ = Zsig_.col(i) - z_pred_;

    //angle normalization
    while (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
    while (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff_ * z_diff_.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R_ = MatrixXd(n_laser_,n_laser_);
  R_ << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
  S_ = S_ + R_;

  /*****************************************************************************
   * UKF Laser Update (Lesson 8.28)
   ****************************************************************************/
  //create example vector for incoming radar measurement
  VectorXd z_ = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc_ = MatrixXd(n_x_, n_laser_);

  //calculate cross correlation matrix
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff_ = Zsig_.col(i) - z_pred_;

    // state difference
    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;

    Tc_ = Tc_ + weights_(i) * x_diff_ * z_diff_.transpose();
  }

  //Kalman gain K;
  MatrixXd K_ = Tc_ * S_.inverse();

  //residual
  VectorXd z_diff_ = z_ - z_pred_;

  //create nis for laser
  NIS_laser_ = z_diff_.transpose() * S_.inverse() * z_diff_;

  //update state mean and covariance matrix
  x_ = x_ + K_ * z_diff_;
  P_ = P_ - K_*S_*K_.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  /*****************************************************************************
   *  Measurement Prediction (Lesson 8.25)
   ****************************************************************************/
  //create matrix for sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(n_radar_, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig_(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig_(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred_ = VectorXd(n_radar_);
  z_pred_.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++)
      z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);

  //innovation covariance matrix S
  MatrixXd S_ = MatrixXd(n_radar_,n_radar_);
  S_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff_ = Zsig_.col(i) - z_pred_;

    //angle normalization
    while (z_diff_(1)> M_PI) z_diff_(1)-=2.*M_PI;
    while (z_diff_(1)<-M_PI) z_diff_(1)+=2.*M_PI;

    S_ = S_ + weights_(i) * z_diff_ * z_diff_.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R_ = MatrixXd(n_radar_,n_radar_);
  R_ <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S_ = S_ + R_;

  /*****************************************************************************
   * UKF Radar Update (Lesson 8.28)
   ****************************************************************************/
  //create example vector for incoming radar measurement
  VectorXd z_ = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc_ = MatrixXd(n_x_, n_radar_);

  //calculate cross correlation matrix
  Tc_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff_ = Zsig_.col(i) - z_pred_;

    //angle normalization
    while (z_diff_(1)> M_PI) z_diff_(1) -= 2.*M_PI;
    while (z_diff_(1)<-M_PI) z_diff_(1) += 2.*M_PI;

    // state difference
    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;

    //angle normalization
    while (x_diff_(3)> M_PI) x_diff_(3) -= 2.*M_PI;
    while (x_diff_(3)<-M_PI) x_diff_(3) += 2.*M_PI;

    Tc_ = Tc_ + weights_(i) * x_diff_ * z_diff_.transpose();
  }

  //Kalman gain K;
  MatrixXd K_ = Tc_ * S_.inverse();

  //residual
  VectorXd z_diff_ = z_ - z_pred_;

  //angle normalization
  while (z_diff_(1)> M_PI) z_diff_(1) -= 2.*M_PI;
  while (z_diff_(1)<-M_PI) z_diff_(1) += 2.*M_PI;

  //create nis for radar
  NIS_radar_ = z_diff_.transpose() * S_.inverse() * z_diff_;

  //update state mean and covariance matrix
  x_ = x_ + K_ * z_diff_;
  P_ = P_ - K_*S_*K_.transpose();

}