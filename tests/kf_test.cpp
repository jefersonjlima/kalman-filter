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
#include <random>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{

  int n = 3; // Number of states
  int m = 1; // Number of measurements
  int c = 1; // Number of control inputs

  std::random_device rd;
  std::mt19937 rgen(rd());
  std::uniform_real_distribution<> R_dis(-0.5, 0.5);

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

  A << 1.1269, -0.4940, 0.1129,
       1.0,    0.0,     0.0,
       0.0,    1.0,     0.0;

 B << -0.3832, 0.5919, 0.5191;
 C << 1.0, 0.0, 0.0;

  mu_0 << 0.0, 0.0, 0.0;
  z << 0.1;
  u << 0.0;

  Sigma_0 << 1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0;

  Q << 2.3, 0.0, 0.0,
       0.0, 2.3, 0.0,
       0.0, 0.0, 2.3;

  R << 1.0;

  cout << "A: \n" << A << endl;
  cout << "B: \n" << B << endl;
  cout << "C: \n" << C << endl;

  cout << "R: \n" << R << endl;
  cout << "Q: \n" << Q << endl;

  cout << "mu_0: \n" << mu_0 << endl;
  cout << "Sigma_0: \n" << Sigma_0 << endl;

  KFilter<Linear> Robot(A, B, C, Q, R);
  Robot.init(mu_0, Sigma_0);

  for (int t = 0; t < 10; t++)
  {

    cout << setprecision(2) << "Time: " << t << "s" << endl;

    //prediction
    u << sin(t/5);
    Robot.time_update(u);

    //update
    z << Robot.mu_hat[0] + R_dis(rgen);
    Robot.measurement_update(z);

    cout << "x_0: " << Robot.mu_hat[0] << endl;
    cout << "x_1: " << Robot.mu_hat[1] << endl;
    cout << "x_2: " << Robot.mu_hat[2] << endl;
  }

  return 0;
}
