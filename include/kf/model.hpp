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

typedef Eigen::MatrixXd Mxd;
typedef Eigen::VectorXd Vxd;
typedef std::function<std::tuple<Mxd,Mxd>(const Vxd&, 
    		int,int,int)> dFdx;

/* @brief Linear Model */
class Linear
{
  int n_, m_, c_;

public:
  bool is_linear = true;

  Linear(
      const Eigen::MatrixXd &A,
      const Eigen::MatrixXd &B,
      const Eigen::MatrixXd &Q);

  Linear(const Eigen::MatrixXd &Q, int n, int m, int c);

  ~Linear(){};

  void init();

  void init(const Eigen::VectorXd &mu_0, Eigen::MatrixXd &Sigma_0);

  void time_update(const Eigen::MatrixXd &u);

//  void h(std::function<Eigen::MatrixXd<Eigen::MatrixXd> 

  //variables
  Eigen::MatrixXd A, B, Q;

  Eigen::VectorXd mu_hat;

  Eigen::MatrixXd Sigma_hat;
};

/* @brief Nonlinear Model */
class NonLinear : public Linear
{
  int n_, m_, c_;
  dFdx g_mu_;

public:
  NonLinear(const Eigen::MatrixXd &Q, int n, int m, int c);
  ~NonLinear(){};

  /* load model */
  void loadModelEqs(const dFdx &g);
  
  //TODO use Automatic differentiation
  void applyModelJacobian(void);
};

#endif
