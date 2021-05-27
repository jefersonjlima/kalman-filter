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

/* @brief Linear Model */
class Linear{

    Eigen::MatrixXd A, B, Q;

    int n;

    public:
      
        Linear(  
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B, 
            const Eigen::MatrixXd& Q
        );

        ~Linear() {};

        void init();

        void init(const Eigen::VectorXd& mu_0, Eigen::MatrixXd& Sigma_0);

        void time_update(const Eigen::MatrixXd& u);

        Eigen::VectorXd mu_hat;

        Eigen::MatrixXd Sigma_hat;

};


/* @brief Nonlinear Model */
class NonLinear : public Linear
{

    public:
      
        NonLinear(const Eigen::MatrixXd& Q);

        ~NonLinear() {};

        void time_update(const Eigen::MatrixXd& u);

        /* Calc Jacobian Matrix */
        virtual void approx_update() {};
};

#endif
