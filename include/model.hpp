/* ----------------------------------------------------------------------------
 * Copyright 2021, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   model.hpp
 *  @author Jeferson Lima
 *  @brief  Header file for Dynamic Model
 *  @date   April 07, 2021
 **/

#ifndef MODEL_HPP_
#define MODEL_HPP_
#include <Eigen/Dense>
#include <iostream>

using namespace std;

class DynSystem{

    Eigen::MatrixXd A, B, C;

    int m, n, c;

    double dt;
    
    Eigen::VectorXd x_hat, mu_hat;
    
    public:

    DynSystem(  
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B, 
            const Eigen::MatrixXd& C,
            double dt
    );

    ~DynSystem() {};

    void init();

    void init(const Eigen::VectorXd& x0);
};
#endif
