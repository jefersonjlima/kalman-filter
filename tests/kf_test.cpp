/* ----------------------------------------------------------------------------
 * Copyright 2021, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ekf_test.cpp
 *  @author Jeferson Lima
 *  @brief  Test file for Extended Kalman Filter
 *  @date   April 07, 2021
 **/

#include "kf.hpp"
#include <iomanip>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{

  int n = 3; // Number of states
  int m = 1; // Number of measurements
  int c = 1; // Number of control inputs

  double dt = 1.0 / 30; // Time step

  MatrixXd A(n, n); // System dynamics matrix
  MatrixXd B(n, c); // Input control matrix
  MatrixXd C(m, n); // Output matrix

  //initial conditions
  VectorXd mu_0(n, m);
  MatrixXd z(m, m);
  MatrixXd u(c, c);
  MatrixXd Sigma_0(n, n);
  MatrixXd Q(n, n); //covariance of the process noise;
  MatrixXd R(m, m); //covariance of the observation noise;

  // Model
  A << 1.0, dt, 0.0,
      0.0, 1.0, dt,
      0.0, 0.0, 1.0;
  B << 0.0, 0.0, 0.0;
  C << 1.0, 0.0, 0.0;

  mu_0 << 1.0, 0.0, 0.0;
  z << 0.1;
  u << 0.3;

  Sigma_0 << 1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0;

  Q << 1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0;

  R << 0.01;

  cout << "A: \n" << A << endl;
  cout << "B: \n" << B << endl;
  cout << "C: \n" << C << endl;

  cout << "R: \n" << R << endl;
  cout << "Q: \n" << Q << endl;

  cout << "mu_0: \n" << mu_0 << endl;
  cout << "Sigma_0: \n" << Sigma_0 << endl;

  KFilter<Linear> Robot(A, B, C, Q, R);
  Robot.init(mu_0, Sigma_0);

  for (int i = 0; i < 10; i++)
  {

    cout << setprecision(2) << "Time: " << i * dt << "s" << endl;

    //prediction
    Robot.time_update(u);

    //update
    Robot.measurement_update(z);

    cout << "x_0: " << Robot.mu_hat[0] << endl;
    cout << "x_1: " << Robot.mu_hat[1] << endl;
  }

  return 0;
}