#include <iostream>
#include "tools.h"

#define MIN_VAL 0.0001
#define MIN_VAL1 0.0000001
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
  for(unsigned int i=0; i < estimations.size(); ++i){

    VectorXd residual = estimations[i] -
      ground_truth[i];

    //coefficient-wise multiplication
    residual =
      residual.array()*residual.array();
    rmse += residual;
  }

  rmse = rmse/estimations.size(); //mean
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation

  if (fabs(px) < MIN_VAL)
    px = MIN_VAL;

  if (fabs(py) < MIN_VAL)
    py = MIN_VAL;

  float c1 = px*px+py*py;
  //check division by zero
  if(fabs(c1) < MIN_VAL){
    c1 = MIN_VAL;
  }


  float c2 = sqrt(c1);

  if(fabs(c2) < MIN_VAL){
    c2 = MIN_VAL;
  }

  float c3 = (c1*c2);


  if(fabs(c3) < MIN_VAL){
    c3 = MIN_VAL;
  }


  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
     -(py/c1), (px/c1), 0, 0,
     py*(vx*py - vy*px)/c3,
     px*(px*vy - py*vx)/c3,
     px/c2, py/c2;

  return Hj;
}


float Tools::normalize_to_pi(float angle){

  while(angle >M_PI)
    angle -= 2*M_PI;

  while(angle < -M_PI)
    angle += 2*M_PI;

  assert(angle > -M_PI && angle < M_PI);
  return angle;
}
