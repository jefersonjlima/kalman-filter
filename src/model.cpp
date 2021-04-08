/* ----------------------------------------------------------------------------
 * Copyright 2021, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ekf.h
 *  @author Jeferson Lima
 *  @brief  Source file for Dynamic Model
 *  @date   April 07, 2021
 **/

#include "model.hpp"

DynSystem::DynSystem(  
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B, 
    const Eigen::MatrixXd& C,
    double dt)
    : A(A), B(B), C(C), dt(dt),
    m(C.rows()), n(A.rows()), c(B.cols()),
    x_hat(n), mu_hat(n) {}

void DynSystem::init(const Eigen::VectorXd& x0){
    x_hat = x0;
}

void DynSystem::init(){
    x_hat.setZero();
}  