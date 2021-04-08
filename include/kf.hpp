/* ----------------------------------------------------------------------------
 * Copyright 2021, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   ekf.hpp
 *  @author Jeferson Lima
 *  @brief  Header file for Extended Kalman Filter
 *  @date   April 07, 2021
 **/

#ifndef EKF_HPP_
#define EKF_HPP_
#include "model.hpp"
#include <Eigen/Dense>


class KFilter : public LinearSystem{

    Eigen::MatrixXd C, R, I;

    int n;
  
    public:

        KFilter(const Eigen::MatrixXd& A,
                const Eigen::MatrixXd& B, 
                const Eigen::MatrixXd& C,
                const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& R
                );

        ~KFilter() {};

        void measurement_update(const Eigen::MatrixXd& z);
};

#endif
