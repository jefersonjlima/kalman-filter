/* ----------------------------------------------------------------------------
 * Copyright 2021, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ekf.h
 *  @author Jeferson Lima
 *  @brief  Source file for Extended Kalman Filter
 *  @date   April 07, 2021
 **/

#include "kf.hpp"
#include <iostream>


KFilter::KFilter(
               const Eigen::MatrixXd& A,
               const Eigen::MatrixXd& B, 
               const Eigen::MatrixXd& C,
               const Eigen::MatrixXd& Q,
               const Eigen::MatrixXd& R
               ) : LinearSystem(A, B, Q), C(C), R(R),
               n(A.rows())
     {
	     I.setIdentity(n,n);
     }

void KFilter::measurement_update(const Eigen::MatrixXd& z){

     auto K = Sigma_hat * C.transpose() * (C * Sigma_hat * C.transpose() + R).inverse();
     mu_hat += K * (z - C * mu_hat);
     Sigma_hat = (I - K * C) * Sigma_hat;
}