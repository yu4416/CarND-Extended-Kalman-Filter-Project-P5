#include "kalman_filter.h"
#include <math.h> // for arctan atan2() and power pow()
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using std::cout;
using std::endl;
using std::vector;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

Eigen::MatrixXd I_;  // Identity matrix

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
  /**
   * TODO: predict the state
   */
  //std::cout<< "x initial: " << x_ << endl;
  //cout<< "P initial: " << P_ << endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  //cout<< "x predict: " << x_ << endl;
  //cout<< "P predict: " << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  // update calculation y = z - Hx'
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  // update measurement, from Lesson 24
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  // predicted measurement vector x'
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  //converting x' from Cartesian coordinates to polar coordinates
  float rho = sqrt( pow(px,2) + pow(py,2) );
  if ( fabs(rho) < 0.0001)
  {
    rho = 0.0001; //checking is rho value is too small and not dividing by 0
  }
  float phi = atan2( py,px );
  float rho_dot= ( px * vx + py * vy ) / rho;

  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;
  VectorXd y = z - h; // adapted equation for radar: y = z_radar - h(x')
  
  //Tips from the project description, normalizing angles, adjusting atan2() values between -pi and pi
  //By adding 2pi or substracting 2pi until the angle is within the desired range
  //This part of the code is adapting from dkarunakaran's Github page, reference: https://github.com/dkarunakaran/carnd-extended-kalman-filter-term2-p1/blob/master/src/kalman_filter.cpp
  while (y(1) < -M_PI)
	  y(1) += 2.0 * M_PI;
  while (y(1) > M_PI)
	  y(1) -= 2.0 * M_PI;
  
  // update measurement, from Lesson 24
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}