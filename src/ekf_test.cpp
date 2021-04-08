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

#include "model.hpp"

class myRobot : public DynSystem{

    int value;
    public:
        double i;
        myRobot(const Eigen::MatrixXd& A,
                const Eigen::MatrixXd& B, 
                const Eigen::MatrixXd& C,
                double dt) : DynSystem(A, B, C, dt){
            i = dt;
            std::cout <<"test" << std::endl;
        }
};

using namespace std;

int main(int argc, char** argv){

    int n = 3; // Number of states
    int m = 1; // Number of measurements
    int c = 1; // Number of control inputs


    double dt = 1.0/30; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix
    Eigen::MatrixXd B(n, c); // Input control matrix
    Eigen::MatrixXd C(m, n); // Output matrix

    A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
    B << 0, 0, 0;
    C << 1, 0, 0;

    std::cout << "A: \n" << A << std::endl;
    std::cout << "B: \n" << B << std::endl;
    std::cout << "C: \n" << C << std::endl;

    myRobot DesertFox(A, B, C, dt);

    return 0;
}