/*
 * GMM.h
 *
 *  Created on: 13-Dec-2014
 *      Author: mobman
 */

#ifndef GMM_H_
#define GMM_H_

//#define pi 3.14

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>

#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <samlibs.h>

#include <iostream>
#include </usr/include/eigen3/Eigen/Core>
#include <cmath>
#include <boost/math/constants/constants.hpp>
#include </usr/include/eigen3/Eigen/LU>

using namespace barrett;
using namespace systems;

template<size_t DOF>
class GMM: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

//public:
//	double pi = boost::math::constants::pi<double>();

public:
	typedef Eigen::Matrix<double, 7, 1> Vector7d;
	Input<jp_type> jpInputActual;
	Input<jv_type> jvInputActual;
public:
	Output<jt_type> error_output;

protected:
	typename Output<jt_type>::Value* error_outputValue;

public:
	GMM(Eigen::VectorXd priors, Eigen::MatrixXd mean, Eigen::MatrixXd sigma,
			size_t num_inp, size_t num_op, size_t num_priors,
			Eigen::MatrixXd sigma_input, Eigen::MatrixXd input_mean_matrix,
			Eigen::MatrixXd A_matrix, Eigen::MatrixXd B_matrix,
			Vector7d det_tmp_tmp, Eigen::MatrixXd tmp_inv,
			const std::string& sysName = "GMM") :
			System(sysName), jpInputActual(this), jvInputActual(this), error_output(
					this, &error_outputValue), priors(priors), mean(mean), sigma(
					sigma), num_inp(num_inp), num_op(num_op), num_priors(
					num_priors), sigma_input(sigma_input), input_mean_matrix(
					input_mean_matrix), A_matrix(A_matrix), B_matrix(B_matrix), det_tmp_tmp(
					det_tmp_tmp), tmp_inv(tmp_inv) {

	}
	virtual ~GMM() {
		this->mandatoryCleanUp();
	}

protected:
	Eigen::VectorXd inputjpjv;
	Eigen::VectorXd priors;/*!< an Eigen Vector value */
	Eigen::MatrixXd mean;/*!< an Eigen Matrix value */
	Eigen::MatrixXd sigma;/*!< an Eigen Matrix value */
	Eigen::VectorXd f_hat;
	jt_type tmp_jt;

public:
	Eigen::VectorXd output;

private:
	size_t num_inp, num_op, num_priors;
	Eigen::MatrixXd sigma_input;
	Eigen::MatrixXd input_mean_matrix;
	Eigen::MatrixXd A_matrix;
	Eigen::MatrixXd B_matrix;
//	Vector7d det_tmp_tmp;
	Eigen::VectorXd det_tmp_tmp;
	Eigen::MatrixXd tmp_inv;
	Eigen::VectorXd p;
	Eigen::VectorXd h;
//	Vector7d p; // FOR SPEED
//	Vector7d h; // FOR SPEED
//	double pi;

protected:
	virtual void operate() {
////		pi = boost::math::constants::pi<double>();
//
		inputjpjv.resize(DOF * 2, 1); /*!Has to be scaled 10 times*/
		inputjpjv[0] = 10 * this->jpInputActual.getValue()[0];
		inputjpjv[1] = 10 * this->jpInputActual.getValue()[1];
		inputjpjv[2] = 10 * this->jpInputActual.getValue()[2];
		inputjpjv[3] = 10 * this->jpInputActual.getValue()[3];
		inputjpjv[4] = 10 * this->jvInputActual.getValue()[0];
		inputjpjv[5] = 10 * this->jvInputActual.getValue()[1];
		inputjpjv[6] = 10 * this->jvInputActual.getValue()[2];
		inputjpjv[7] = 10 * this->jvInputActual.getValue()[3];
//
//		//-----------------------------------SECTION ONE-----------------------------------------------------//
//
		int num_priors = priors.rows(); // priors comes as a rows
		int num_inp = 8; //inputjpjv.rows();   ----------------------------------SHOULDNOT DO THIS ------------//
		int k = 0;
		p.resize(num_priors, 1);
		h.resize(num_priors, 1);
		long double prob_input = 0;
//		//-------------------------------SECTION TWO---------------------------------------------------//
//
		for (k = 0; k < num_priors; k++) {
			double coeff = pow((2 * 3.14), (inputjpjv.rows()) / 2)   // replace pi by 3.14
					* det_tmp_tmp[k];
			double exponential_part = (-0.5
					* (((inputjpjv - input_mean_matrix.col(k)).transpose())
							* (tmp_inv.block(num_inp * k, 0, num_inp, num_inp)))
					* (inputjpjv - input_mean_matrix.col(k)))(0, 0);

			double result = (1 / coeff) * exp(exponential_part);

			double prob_input_tmp = result;/*gausspdf(inputjpjv,*/
			double tmp_Pk = priors(k) * prob_input_tmp;
			p(k) = tmp_Pk;
			prob_input = prob_input + tmp_Pk;

		}
//
//		//SUB
//
//		p[0] = 0.14;
//		p[1] = 0.14;
//		p[2] = 0.14;
//		p[3] = 0.14;
//		p[4] = 0.14;
//		p[5] = 0.14;
//		p[6] = 0.16;
//		prob_input = 1.0;
//
//		//-------------------------------SECTION THREE---------------------------------------------------//
//
		for (k = 0; k < num_priors; k++) {
			double h_k = p(k) / prob_input;
			h(k) = h_k;
		}
//
//		h[0] = 0.14;
//		h[1] = 0.14;
//		h[2] = 0.14;
//		h[3] = 0.14;
//		h[4] = 0.14;
//		h[5] = 0.14;
//		h[6] = 0.16;
//		//-------------------------------SECTION FOUR---------------------------------------------------//
//

		f_hat.resize(num_op, 1);
		f_hat.fill(0.0);


		for (k = 0; k < num_priors; k++) {
			f_hat = f_hat
					+ h(k)
							* (A_matrix.block(num_op * k, 0, num_op, num_inp)
									* inputjpjv + B_matrix.col(k));
		}

//		f_hat.resize(num_op, 1);
//
//		f_hat[0] = 0.5;
//		f_hat[1] = 0.5;
//		f_hat[2] = 0.5;
//		f_hat[3] = 0.5;
//
//		//-------------------------------SECTION FIVE---------------------------------------------------//
//
		tmp_jt[0] = f_hat[0];
		tmp_jt[1] = f_hat[1];
		tmp_jt[2] = f_hat[2];
		tmp_jt[3] = f_hat[3];
//		tmp_jt[0] = 0;
//		tmp_jt[1] = 0;
//		tmp_jt[2] = 0;
//		tmp_jt[3] = 0;
		this->error_outputValue->setData(&tmp_jt);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(GMM);
};

//#include <Detail/GMM-inl.h>
#endif /* GMM_H_ */
