#include "tools.h"
#include "kalman_filter.h"
#define THRESHOLD_VAL 0.001
#define THRESHOLD_VAL_SQROOT 0.00001
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;
KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:Predict enter"<<endl;
#endif
   x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:Predict "<<endl << P_<<endl;
#endif
}

/* This update function is common between EKF and KF */
void KalmanFilter::KFCommonUpdate(const VectorXd &y){

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:KFCommonUpdate enter"<<endl;
#endif
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:KFCommonUpdat exit"<<endl;
#endif
}

void KalmanFilter::Update(const VectorXd &z) {
#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:Update enter"<<endl;
#endif
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  KFCommonUpdate(y);

#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:Update exit"<<endl;
#endif
  }

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  Tools tools;
  VectorXd h(3);

  if(fabs(x_(0)) < THRESHOLD_VAL){
    x_(0) = THRESHOLD_VAL;
  }

  if(fabs(x_(1)) < THRESHOLD_VAL){
    x_(1) = THRESHOLD_VAL;
  }

  float rho = sqrt(x_(0) * x_(0) + x_(1)*x_(1));

  if(fabs(rho) < THRESHOLD_VAL){
    rho = THRESHOLD_VAL;
  }

  float theta = atan2(x_(1),x_(0));

  assert(theta > -M_PI && theta < M_PI);
  assert(rho > 0.0);

  float rho_d= (x_(0)*x_(2) + x_(1)*x_(3))/rho;

#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:Update enter"<<endl;
#endif
  h << rho,theta,rho_d;
  VectorXd y = z - h;

  y(1) = tools.normalize_to_pi(y(1));

  KFCommonUpdate(y);

#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:UpdateEKF exit"<<endl;
#endif
}
