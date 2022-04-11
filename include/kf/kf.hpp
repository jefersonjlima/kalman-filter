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

#ifndef KF_HPP_
#define KF_HPP_
#include "model.hpp"

template <class T>
class KFilter : public T
{
  int n_, m_, c_;
  dFdx h_mu_;
  Eigen::MatrixXd h_;

public:
  KFilter(const Eigen::MatrixXd &A,
          const Eigen::MatrixXd &B,
          const Eigen::MatrixXd &C,
          const Eigen::MatrixXd &Q,
          const Eigen::MatrixXd &R) : T(A, B, Q), C(C), R(R), n_(A.rows())
  {
    I.setIdentity(n_, n_);
  }

  KFilter(const Eigen::MatrixXd &R, const Eigen::MatrixXd &Q, int n, int m, int c)
      : T(Q, n, m, c), R(R), n_(n), m_(m), c_(c)
  {
    I.setIdentity(n_, n_);
    C = Eigen::MatrixXd(m_, n_).setZero();
  }

  ~KFilter(){};

  void measurement_update(const Eigen::MatrixXd &z)
  {
    auto K = this->Sigma_hat * C.transpose() * (C * this->Sigma_hat * C.transpose() + R).inverse();
    if (this->is_linear)
    {
    	this->mu_hat = this->mu_hat + K * (z - (C * this->mu_hat));
    }
    else
    	this->mu_hat = this->mu_hat + K * (z - h_);

    this->Sigma_hat = (I - K * C) * this->Sigma_hat;
  }

  void loadSensorEqs(const dFdx& h)
  {
    h_mu_ = h;
  }

  void applySensorJacobian()
  {
    std::tie(C, h_) = h_mu_(this->mu_hat,n_, m_, c_);
  }

  Eigen::MatrixXd R, C, I;

};

#endif
