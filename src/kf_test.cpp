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


// class myRobot : public KFilter{

//     public:
//         double i;
//         myRobot(const Eigen::MatrixXd& A,
//                 const Eigen::MatrixXd& B, 
//                 const Eigen::MatrixXd& C,
//                 const Eigen::MatrixXd& Q
//                 ) : KFilter(A, B, C, Q) {}
// };

using namespace std;


int main(int argc, char** argv){

    int n = 3; // Number of states
    int m = 1; // Number of measurements
    int c = 1; // Number of control inputs


    double dt = 1.0/30; // Time step

    Eigen::MatrixXd A(n, n);    // System dynamics matrix
    Eigen::MatrixXd B(n, c);    // Input control matrix
    Eigen::MatrixXd C(m, n);    // Output matrix
    
    //initial conditions
    Eigen::VectorXd mu_0(n,m);
    Eigen::MatrixXd z(m,m);
    Eigen::MatrixXd u(c,c);
    Eigen::MatrixXd Sigma_0(n,n);  
    Eigen::MatrixXd Q(n,n);     //covariance of the process noise;
    Eigen::MatrixXd R(m,m);     //covariance of the observation noise;

    A << 1, dt, 0,
         0, 1, dt,
          0, 0, 1;
    B << 0, 0, 0;
    C << 1, 0, 0;

    mu_0 << 1, 0 , 0;
    z << 0.1;
    u << 0.3;

    Sigma_0 <<  1, 0 , 0,
             0, 1 , 0,
             0, 0 , 1;
    
    Q << 1, 0 , 0,
         0, 1 , 0,
         0, 0 , 1;

    R << 0.01;

    std::cout << "A: \n" << A << std::endl;
    std::cout << "B: \n" << B << std::endl;
    std::cout << "C: \n" << C << std::endl;

    std::cout << "R: \n" << R << std::endl;
    std::cout << "Q: \n" << Q << std::endl;

    std::cout << "mu_0: \n" << mu_0 << std::endl;
    std::cout << "Sigma_0: \n" << Sigma_0 << std::endl;


    KFilter DesertFox(A, B, C, Q, R);
    DesertFox.init(mu_0, Sigma_0);

    for (int i = 0; i < 100; i++){

        std::cout << std::setprecision(2) << "Time: " << i * dt << "s" << endl;

        //prediction
        DesertFox.time_update(u);

        //update
        DesertFox.measurement_update(z);

        std::cout << "mu: \n" << DesertFox.mu_hat << std::endl;
    }

    return 0;
}