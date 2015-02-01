/*
 * GMM.h
 *
 *  Created on: 13-Dec-2014
 *      Author: mobman
 */

#ifndef GMM_H_
#define GMM_H_
#include <iostream>
#include <samlibs.h>
#include </usr/include/eigen3/Eigen/Core>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include </usr/include/eigen3/Eigen/LU>
namespace isl {

  class GMM {
  public:
    GMM(double Rho_0, double K);
    virtual  ~GMM();


  protected:
    Eigen::VectorXd priors;/*!< an Eigen Vector value */
    Eigen::MatrixXd mean;/*!< an Eigen Matrix value */
    Eigen::MatrixXd sigma;/*!< an Eigen Matrix value */
    Eigen::VectorXd xistar_trn;
    Eigen::MatrixXd P_0_data;/*!< Symmetric Lyapunov component */
    Eigen::MatrixXd P_total_data;/*!< matrix containing all asymmetric components */
    Eigen::VectorXd Mu;
    Eigen::VectorXd f_hat;
    Eigen::VectorXd grad_val;

  public:
    Eigen::VectorXd output;
#ifdef DEBUG_GMM_
    Eigen::Matrix<double, 3,1> uOutput, fOutput;
#endif
  private:
    size_t num_inp, num_op, num_priors, num_lyp_asym_copont;
    double RHO_0, K_;
    double pi;
  private:
    void f_estimate(const Eigen::VectorXd& input);
    void calculate_gradient(const Eigen::VectorXd& input);
    Eigen::VectorXd control_input(const Eigen::VectorXd& input);
    double gausspdf(const Eigen::VectorXd& input, const Eigen::VectorXd& Mean, const Eigen::MatrixXd& Sigma);
  public:
    void calculate_model_output(const Eigen::VectorXd& input);

  };



} /* namespace isl */
#include <Detail/GMM-inl.h>
#endif /* GMM_H_ */
