#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define MIN_VAL 0.0001
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
  H_laser_ << 1,0,0,0,
             0,1,0,0;

  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
           0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
           0, 0.0009, 0,
           0, 0, 0.09;
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
TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    /* cout << "EKF:" << measurement_pack.raw_measurements_ << endl; */
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
        Convert radar from polar to cartesian coordinates and initialize state.
        */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_d= measurement_pack.raw_measurements_[2];

      /* x is projection of the rho vector on x axis */
      float x =  rho * cos(phi);
      /* x is projection of the rho vector on y axis */
      float y =  rho * sin(phi);

      /* Similiarly, velocity projections on x and y axis */
      float vx = rho_d * cos(phi);
      float vy = rho_d * sin(phi);

      ekf_.x_ << x,y,vx,vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
        Initialize state.
        */
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;
    }

    if(ekf_.x_[0] < MIN_VAL)
      ekf_.x_[0] = MIN_VAL;

    if(ekf_.x_[1] < MIN_VAL)
      ekf_.x_[1] = MIN_VAL;

    /* cout <<"EKF start value "<< ekf_.x_ << endl; */
    ekf_.P_ = MatrixXd(4,4);

    ekf_.P_ << 1,0,0,0,
      0,1,0,0,
      0,0,1000,0,
      0,0,0,1000;

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
TODO:
   * Update the state transition matrix F according to the new elapsed time.
   - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  /* dt in micro seconds */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,dt,0,
    0,1,0,dt,
    0,0,1,0,
    0,0,0,1;

  float ax = 9.0,ay = 9.0;
  const float dt_2 = dt * dt;
  const float dt_3 = dt_2 * dt;
  const float dt_4 = dt_3 * dt;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt_4*ax/4, 0,        dt_3*ax/2,   0,
    0,           dt_4*ay/4,0,           dt_3*ay/2,
    dt_3*ax/2,   0,        dt_2 * ax,     0,
    0,           dt_3*ay/2,0,           dt_2 * ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
TODO:
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else {
    // Laser updates
    //
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  /* cout << "x_ = " << ekf_.x_ << endl; */
  /* cout << "P_ = " << ekf_.P_ << endl; */
}
