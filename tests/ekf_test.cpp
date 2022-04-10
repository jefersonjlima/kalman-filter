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

#include <kf/kf.hpp>
#include <iomanip>

using namespace std;
using namespace Eigen;

std::tuple<MatrixXd, MatrixXd, VectorXd> myRobot(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const VectorXd &mu)
{

  MatrixXd _A(3,3);
 _A << mu(0)    , mu(1)    , mu(2),
       0        , 0        , 1,
       0         ,0        , 1;

 MatrixXd z(1, 1);

  z << 0.1;

  return {_A, B, z};
}

int main(int argc, char **argv)
{

  int n = 3; // Number of states
  int m = 1; // Number of measurements
  int c = 1; // Number of control inputs

  double dt = 1.0 / 30; // Time step

  //initial conditions
  VectorXd mu_0(n, m);
  MatrixXd z(m, m);
  MatrixXd u(c, c);
  MatrixXd Sigma_0(n, n);
  MatrixXd Q(n, n); //covariance of the process noise;
  MatrixXd R(m, m); //covariance of the observation noise;

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

  cout << "R: \n" << R << endl;
  cout << "Q: \n" << Q << endl;

  cout << "mu_0: \n" << mu_0 << endl;
  cout << "Sigma_0: \n" << Sigma_0 << endl;

  KFilter<NonLinear> Robot(R, Q, m, c);
  Robot.loadEq(myRobot);
  Robot.init(mu_0, Sigma_0);

  auto mu = mu_0;
  for (int i = 0; i < 10; i++)
  {

    cout << setprecision(2) << "Time: " << i * dt << "s" << endl;

    //linearization
    Robot.applyTaylorJacobian();

    //prediction
    Robot.time_update(u);

    //update
    Robot.measurement_update(z);

    cout << "x_0: " << Robot.mu_hat[0] << endl;
    cout << "x_1: " << Robot.mu_hat[1] << endl;
  }

  return 0;
}
