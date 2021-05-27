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

// template<class T>
// void KFilter<T>::measurement_update(const Eigen::MatrixXd& z)
// {
//      auto K = this->Sigma_hat * C.transpose() * (C * this->Sigma_hat * C.transpose() + R).inverse();
//      this->mu_hat += K * (z - C * this->mu_hat);
//      this->Sigma_hat = (I - K * C) * this->Sigma_hat;
// }