/*
 * GMM.h
 *
 *  Created on: 13-Dec-2014
 *      Author: mobman
 */

#ifndef GMM_H_
#define GMM_H_

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

public:
	Input<jp_type> jpInputActual;
	Input<jv_type> jvInputActual;
public:
	Output<jt_type> error_output;

protected:
	typename Output<jt_type>::Value* error_outputValue;

public:
	GMM(Eigen::VectorXd priors, Eigen::MatrixXd mean, Eigen::MatrixXd sigma,
			size_t num_inp, size_t num_op, size_t num_priors,
			const std::string& sysName = "GMM") :
			System(sysName), jpInputActual(this), jvInputActual(this), error_output(
					this, &error_outputValue), priors(priors), mean(mean), sigma(
					sigma), num_inp(num_inp), num_op(num_op), num_priors(
					num_priors) {

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
	double pi;

protected:
	virtual void operate() {
		pi = boost::math::constants::pi<double>();

		inputjpjv.resize(DOF * 2, 1); /*!Has to be scaled 10 times*/
		inputjpjv[0] = 10 * this->jpInputActual.getValue()[0];
		inputjpjv[1] = 10 * this->jpInputActual.getValue()[1];
		inputjpjv[2] = 10 * this->jpInputActual.getValue()[2];
		inputjpjv[3] = 10 * this->jpInputActual.getValue()[3];
		inputjpjv[0] = 10 * this->jvInputActual.getValue()[0];
		inputjpjv[1] = 10 * this->jvInputActual.getValue()[1];
		inputjpjv[2] = 10 * this->jvInputActual.getValue()[2];
		inputjpjv[3] = 10 * this->jvInputActual.getValue()[3];

		// First function

		int num_priors = priors.rows(); // priors comes as a rows
		int num_inp = inputjpjv.rows();

		Eigen::MatrixXd input_mean_matrix(num_inp, num_priors); //   input  position(xi) mean vector for 6 gaussians
		Eigen::MatrixXd out_mean_matrix(num_inp, num_priors); //
		Eigen::MatrixXd sigma_input(num_inp * num_priors, num_inp); //  extracted  sigma for input
		Eigen::MatrixXd sigma_output_input(num_inp * num_priors, num_inp); //   extracted sigma sigmadot for input
		Eigen::MatrixXd Inverse_sigma_input(num_inp * num_priors, num_inp); // inverse of the sigma matrix taking 6, 2x2 matrix
		Eigen::VectorXd p(num_priors); //probability of input for 6 components
		Eigen::VectorXd h(num_priors); // h value in the linear regression equation
		Eigen::MatrixXd A_matrix(num_inp * num_priors, num_inp); // 'A' matrix in the linear regression equation
		Eigen::MatrixXd B_matrix(num_inp, num_priors); // 'B' matrix in the linear regression equation
		Eigen::VectorXd output(num_inp); // output 'xi dot' of regression equation

		int i = 0;
		int k = 0;
		//  float f = 0; // denominator value of h(k)

		input_mean_matrix = mean.block(0, 0, num_inp, num_priors);
		out_mean_matrix = mean.block(num_inp, 0, num_inp, num_priors);

		//

		for (i = 0; i < num_priors; i++) {
			sigma_input.block(num_inp * i, 0, num_inp, num_inp) = sigma.block(
					2 * num_inp * i, 0, num_inp, num_inp);
		}

		//
		for (i = 0; i < num_priors; i++) {
			sigma_output_input.block(num_inp * i, 0, num_inp, num_inp) =
					sigma.block(2 * num_inp * i + num_inp, 0, num_inp, num_inp);
		}

		//

		for (i = 0; i < num_inp * num_priors - num_inp; i = i + num_inp) //inverse
				{
			Inverse_sigma_input.block(i, 0, num_inp, num_inp) =
					sigma_input.block(i, 0, num_inp, num_inp).inverse();
		}

		//

		long double prob_input = 0;

		//float prob_input = input_probability(input,mean1,sigma1);
		for (k = 0; k < num_priors; k++) {

			//----------GAUSS PDF------------------//
			double det_tmp_tmp = pow(
					double(
							fabs(
									sigma_input.block(num_inp * k, 0, num_inp,
											num_inp).determinant())), 0.5);
			double coeff = pow((2 * pi), (inputjpjv.rows()) / 2) * det_tmp_tmp;
			double exponential_part = (-0.5
					* (((inputjpjv - input_mean_matrix.col(k)).transpose())
							* (sigma_input.block(num_inp * k, 0, num_inp,
									num_inp).inverse()))
					* (inputjpjv - input_mean_matrix.col(k)))(0, 0);

			double result = (1 / coeff) * exp(exponential_part);

			//---------------------------------------//
			double prob_input_tmp = result;/*gausspdf(inputjpjv,
			 input_mean_matrix.col(k),
			 sigma_input.block(num_inp * k, 0, num_inp, num_inp));*/
			double tmp_Pk = priors(k) * prob_input_tmp;
			p(k) = tmp_Pk;
			prob_input = prob_input + tmp_Pk;

		}

		for (k = 0; k < num_priors; k++) {
			double h_k = p(k) / prob_input;
			h(k) = h_k;
		}

		for (k = 0; k < num_inp * num_priors - num_inp; k = k + num_inp) {
			A_matrix.block(k, 0, num_inp, num_inp) = sigma_output_input.block(k,
					0, num_inp, num_inp)
					* Inverse_sigma_input.block(k, 0, num_inp, num_inp);
		}

		for (k = 0; k < num_priors; k++) {
			B_matrix.col(k) = out_mean_matrix.col(k)
					- A_matrix.block(num_inp * k, 0, num_inp, num_inp)
							* input_mean_matrix.col(k);
		}

		f_hat.fill(0.0);

		for (k = 0; k < num_priors; k++) {
			f_hat = f_hat
					+ h(k)
							* (A_matrix.block(num_inp * k, 0, num_inp, num_inp)
									* inputjpjv + B_matrix.col(k));
		}

		//-----------------------------------------------------------------//

		tmp_jt[0] = f_hat[0];
		tmp_jt[1] = f_hat[1];
		tmp_jt[2] = f_hat[2];
		tmp_jt[3] = f_hat[3];
		this->error_outputValue->setData(&tmp_jt);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(GMM);
};

//#include <Detail/GMM-inl.h>
#endif /* GMM_H_ */
