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
#ifdef USE_MATPLOT
 #include <matplot/matplot.h>
#endif
#include <iomanip>

using namespace std;
using namespace Eigen;
#ifdef USE_MATPLOT
 using namespace matplot;
#endif

int main(int argc, char **argv)
{

  int n = 2; // Number of states
  int m = 1; // Number of measurements
  int c = 1; // Number of control inputs

  std::random_device rd;
  std::mt19937 rgen(rd());
  std::uniform_real_distribution<> R_dis(-0.5, 0.5);
  double deltaT = 0.1; // Ts = 0.5 seconds
  double vt;
#ifdef USE_MATPLOT
  vector<double> sensor, position, k;
#endif
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
  A << 	1.0,	deltaT,
    	0.0, 	1.0;

  B << 	0.0, 	deltaT;
  C << 1.0, 0.0;

  mu_0 << 0.0, 5.0;
  z << 2.2;
  u << -2.0;

  Sigma_0 << 	0.01,	0.0,
  		0.0, 	1.0;

  Q << 	0.1, 	0.0,
    	0.0, 	0.1;

  R << 1.0;

//  cout << "A: \n" << A << endl;
//  cout << "B: \n" << B << endl;
//  cout << "C: \n" << C << endl;
//
//  cout << "R: \n" << R << endl;
//  cout << "Q: \n" << Q << endl;
//
//  cout << "mu_0: \n" << mu_0 << endl;
//  cout << "Sigma_0: \n" << Sigma_0 << endl;

  KFilter<Linear> Car(A, B, C, Q, R);
  Car.init(mu_0, Sigma_0);

  cout << fixed;
  cout << setprecision(3);
  cout << "time\tx1\tx2" <<endl;
  for (int t = 0; t < 40; t++)
  {
    // print time
    cout <<t*deltaT << "\t";
    u << -2.0;
    //prediction
    Car.time_update(u);
    //update
    Car.measurement_update(z);
    //print x1 and x2
    cout << Car.mu_hat[0] << "\t"
      << Car.mu_hat[1]
      << endl;

    //data record
#ifdef USE_MATPLOT
    sensor.push_back(z(0));
    position.push_back(Car.mu_hat[0]);
    k.push_back( (double)t * deltaT);
#endif
    // sensor mesurement
    vt = R_dis(rgen);
    z << Car.mu_hat[0] + vt; 
  }
  //plot
#ifdef USE_MATPLOT
  plot(k, position, "--xr", k, sensor, "-:bs");
  show();
#endif

  return 0;
}
