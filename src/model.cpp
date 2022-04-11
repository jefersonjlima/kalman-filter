/* ----------------------------------------------------------------------------
 * Copyright 2021, Jeferson Lima
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   model.h
 *  @author Jeferson Lima
 *  @brief  Source file for Dynamic Model
 *  @date   April 07, 2021
 **/

#include <kf/model.hpp>

/* @brief Linear Model */
Linear::Linear(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q)
    : A(A), B(B), Q(Q),
      n_(A.rows()), mu_hat(n_), Sigma_hat(n_, n_) {}

Linear::Linear(
    const Eigen::MatrixXd& Q, int n, int m, int c)
    : Q(Q), n_(n), m_(m), c_(c), mu_hat(n_), Sigma_hat(n_, n_)
{
  A = Eigen::MatrixXd(n_, n_).setZero();
  B = Eigen::MatrixXd(n_, c_).setZero();
}

void Linear::init(const Eigen::VectorXd& mu_0, Eigen::MatrixXd& Sigma_0)
{
  mu_hat = mu_0;
  Sigma_hat = Sigma_0;
}

void Linear::init()
{
  mu_hat.setZero();
  Sigma_hat.setZero();
}

void Linear::time_update(const Eigen::MatrixXd& u)
{
  mu_hat = A * mu_hat + B * u;
  Sigma_hat = A * Sigma_hat * A.transpose() + Q;
}

/* @brief Nonlinear Model */
NonLinear::NonLinear(const Eigen::MatrixXd& Q, int n, int m, int c)
    : Linear(Q, n, m, c), n_(n), m_(m), c_(c) 
{
  this->is_linear = false;
}

void NonLinear::loadModelEqs(const dFdx& g_mu)
{
  g_mu_ = g_mu;
}

void NonLinear::applyModelJacobian()
{
  std::tie(A, B) = g_mu_(mu_hat, n_, m_, c_);
}
