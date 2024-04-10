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
#include <random>

using namespace std;
using namespace Eigen;

const double deltaT = 0.1;

std::tuple<MatrixXd, MatrixXd> carModel(
    const VectorXd &mu, int n, int m, int c)
{
  MatrixXd A(n, n);
  MatrixXd B(n, c);
    // Model
  A << 	1.0,	deltaT,
    	0.0, 	1.0;

  B << 	0.0, 	deltaT;
  //not to do here
  return {A, B};
}

std::tuple<MatrixXd, MatrixXd> sensorModel(
    const VectorXd &mu, int n, int m, int c)
{
  const double S{20.0}, D{40.0};
  MatrixXd C(m, n);
  MatrixXd h(m, m);
  C(0, 0) = S / ( std::pow(D-mu[0], 2) + std::pow(S, 2));
  C(0, 1) = 0.0;

  h << std::atan(S / (D-mu[0]));

  return {C, h};
}

int main(int argc, char **argv)
{

  int n = 2; // Number of states
  int m = 1; // Number of measurements
  int c = 1; // Number of control inputs

  std::random_device rd;
  std::mt19937 rgen(rd());
  std::uniform_real_distribution<> R_dis(-0.05, 0.05);
  double vt;
#ifdef USE_MATPLOT
  vector<double> sensor, position, k;
#endif

  //initial conditions
  VectorXd mu_0(n, m);
  MatrixXd z(m, m);
  MatrixXd u(c, c);
  MatrixXd Sigma_0(n, n);
  MatrixXd Q(n, n); //covariance of the process noise;
  MatrixXd R(m, m); //covariance of the observation noise;


  mu_0 << 0.0, 5.0;
  z << M_PI/6;
  u << -2.0;

  Sigma_0 << 	0.01,	0.0,
  		0.0, 	1.0;

  Q << 	0.1, 	0.0,
    	0.0, 	0.1;

  R << 0.01;

  KFilter<NonLinear> Car(R, Q, n, m, c);
  Car.loadModelEqs(carModel);
  Car.loadSensorEqs(sensorModel);
  Car.init(mu_0, Sigma_0);

  auto mu = mu_0;
  for (int t = 0; t < 40; t++)
  {

    cout << (double)t*deltaT << "s \t";

    //linearization
    Car.applyModelJacobian();
    Car.applySensorJacobian();

    //prediction
    Car.time_update(u);

    //update
    Car.measurement_update(z);

    cout << "z: " << z << "\t";
    cout << "x_0: " << Car.mu_hat[0] << "\t";
    cout << "x_1: " << Car.mu_hat[1] << endl;

    // sensor mesurement
    vt = R_dis(rgen);
    z(0,0) = std::atan(20.0 / (40.0 - Car.mu_hat[0])) + vt + 0.001;
  }
  return 0;
}
