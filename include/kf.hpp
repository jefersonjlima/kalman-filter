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

template <class T>
class KFilter : public T
{
  int m, n;

public:
  KFilter(const Eigen::MatrixXd &A,
          const Eigen::MatrixXd &B,
          const Eigen::MatrixXd &C,
          const Eigen::MatrixXd &Q,
          const Eigen::MatrixXd &R) : T(A, B, Q), C(C), R(R), n(A.rows())
  {
    I.setIdentity(n, n);
  }

  KFilter(const Eigen::MatrixXd &R, const Eigen::MatrixXd &Q, int m, int c)
      : T(Q, m, c), R(R), n(Q.rows()), m(m)
  {
    I.setIdentity(n, n);
    C = Eigen::MatrixXd(m, n).setZero();
  }

  ~KFilter(){};

  void measurement_update(const Eigen::MatrixXd &z)
  {
    auto K = this->Sigma_hat * C.transpose() * (C * this->Sigma_hat * C.transpose() + R).inverse();
    this->mu_hat += K * (z - C * this->mu_hat);
    this->Sigma_hat = (I - K * C) * this->Sigma_hat;
  }

  Eigen::MatrixXd R, C, I;

};

#endif
