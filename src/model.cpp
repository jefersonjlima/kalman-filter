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

#include "model.hpp"

/* @brief Linear Model */
Linear::Linear(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q)
    : A(A), B(B), Q(Q),
      n(A.rows()), mu_hat(n), Sigma_hat(n, n) {}

Linear::Linear(
    const Eigen::MatrixXd& Q, int m, int c)
    : Q(Q), m(m), c(c), n(Q.rows()), mu_hat(n), Sigma_hat(n, n)
{
  A = Eigen::MatrixXd(n, n).setZero();
  B = Eigen::MatrixXd(n, c).setZero();
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
NonLinear::NonLinear(const Eigen::MatrixXd& Q, int m, int c)
    : Linear(Q, m, c)
{
}

void NonLinear::loadEq(const taylorJacobian& J)
{
  _J = J;
}

void NonLinear::applyTaylorJacobian()
{
  _J(A,B,mu_hat);

}
