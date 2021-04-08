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

class LinearSystem{

    Eigen::MatrixXd A, B, Q;

    int n;

    public:
      
        Eigen::VectorXd mu_hat;

        Eigen::MatrixXd Sigma_hat;

        LinearSystem(  
                const Eigen::MatrixXd& A,
                const Eigen::MatrixXd& B, 
                const Eigen::MatrixXd& Q
        );

        ~LinearSystem() {};

        void init();

        void init(const Eigen::VectorXd& mu_0, Eigen::MatrixXd& Sigma_0);

        void time_update(const Eigen::MatrixXd& u);
};

#endif
