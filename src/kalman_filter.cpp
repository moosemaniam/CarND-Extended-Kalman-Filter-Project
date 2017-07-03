#include "kalman_filter.h"

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
  std::cout << "KalmanFilter:Predict exit"<<endl;
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
  std::cout << "KalmanFilter:KFCommonUpdat enter"<<endl;
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
  VectorXd h(3);

  float rho = sqrt(x_(0) * x_(0) + x_(1)*x_(1));
  float theta = atan(x_(1)/x_(0));
  float rho_d= (x_(0)*x_(2) + x_(1)*x_(2))/rho;

#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:Update enter"<<endl;
#endif
  h << rho,theta,rho_d;
  VectorXd y = z - h;

  KFCommonUpdate(y);

#ifdef DEBUGGING_CODE
  std::cout << "KalmanFilter:UpdateEKF exit"<<endl;
#endif
}
