#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  /* check the validity of the following inputs: */
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector
    /* size */
    if(estimations.size() != ground_truth.size()
        || estimations.size() == 0){
      cout << "Invalid estimation or ground_truth data" <<
        endl;
      return rmse;
    }

  //accumulate squared residuals
  for(unsigned int i=0; i <
      estimations.size(); ++i){

    VectorXd residual = estimations[i] -
      ground_truth[i];

    //coefficient-wise multiplication
    residual =
      residual.array()*residual.array();
    rmse += residual;
  }

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
     -(py/c1), (px/c1), 0, 0,
     py*(vx*py - vy*px)/c3,
     px*(px*vy - py*vx)/c3,
     px/c2, py/c2;

  return Hj;
}
/* ----------------------------------------------------------------------*/
/**
 * @Description calculates range
 *
 * @Param measurement_pack
 *
 */
/* ----------------------------------------------------------------------*/
float calculate_range(const MeasurementPackage &measurement_pack){
  float px = measurement_pack.raw_measurements_[0];
  float py = measurement_pack.raw_measurements_[1];

  return(sqrt(px*px+py*py));
}
/* ----------------------------------------------------------------------*/
/**
 * @Description
 *
 * @Param measurement_pack
 *
 * @Returns
 */
/* ----------------------------------------------------------------------*/
float calculate_bearing(const MeasurementPackage &measurement_pack){
  float px = measurement_pack.raw_measurements_[0];
  float py = measurement_pack.raw_measurements_[1];

  return(atan(py/px));

}

float calculate_range_rate(const MeasurementPackage &measurement_pack){
  float px = measurement_pack.raw_measurements_[0];
  float py = measurement_pack.raw_measurements_[1];
  float vx = measurement_pack.raw_measurements_[2];
  float vy = measurement_pack.raw_measurements_[3];

  return((px * vx + py * vy)/(sqrt(px*px+py*py)));
}

VectorXd Tools::cartesian_to_polar(const MeasurementPackage &measurement_pack){
  VectorXd polar(3);
  polar(0) = calculate_range(measurement_pack);
  polar(1) = calculate_bearing(measurement_pack);
  polar(2) = calculate_range_rate(measurement_pack);
  return polar;
}
