/*
 * torque_observer.hpp
 *
 *  Created on: 28-Feb-2015
 *      Author: nilxwam
 */

#ifndef TORQUE_OBSERVER_HPP_
#define TORQUE_OBSERVER_HPP_

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

using namespace barrett;
using namespace systems;

template<size_t DOF>
class torque_observer: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<double> time;
	Input<ja_type> inputacc; // the input signal is either a jp_type , jv_type
	Input<Eigen::Matrix4d> M;
	Input<Eigen::Vector4d> C;
	Input<jt_type> SMC;

public:
	Output<jt_type> outputTorque; // the output signal is either a jv_type , jv_type

protected:
	typename Output<jt_type>::Value* outputTorqueOutputValue;

public:
	torque_observer(Eigen::Matrix4d lambda) :
			time(this),inputacc(this), M(this), C(this),SMC(this), outputTorque(this,
					&outputTorqueOutputValue), lambda(lambda) {
		error_estimate = Eigen::Vector4d::Zero();
	}

	virtual ~torque_observer() {
		this->mandatoryCleanUp();
	}

protected:
	Eigen::Matrix4d lambda;
	Eigen::Vector4d error;
	Eigen::Vector4d error_estimate;
	Eigen::Vector4d SMC_torque;
	Eigen::Vector4d inputacc_i; // the input signal is either a jp_type , jv_type
	Eigen::Matrix4d M_i;
	Eigen::Vector4d C_i;
	Eigen::Vector2d time_vector;
	double del_time;
	jt_type tmp_error_torque;


	virtual void operate() {


		time_vector[0] = time_vector[1];
		time_vector[1] = this->time.getValue();
		del_time = time_vector[0] - time_vector[1];


		SMC_torque[0] = this->SMC.getValue()[0];
		SMC_torque[1] = this->SMC.getValue()[1];
		SMC_torque[2] = this->SMC.getValue()[2];
		SMC_torque[3] = this->SMC.getValue()[3];

		inputacc_i[0] = this->inputacc.getValue()[0];
		inputacc_i[1] = this->inputacc.getValue()[1];
		inputacc_i[2] = this->inputacc.getValue()[2];
		inputacc_i[3] = this->inputacc.getValue()[3];

		M_i = this->M.getValue();
		C_i = this->C.getValue();

		error = SMC_torque - M_i*inputacc_i - C_i;
		error_estimate = error_estimate + lambda*(error - error_estimate)*del_time;

		tmp_error_torque[0] = error_estimate[0];
		tmp_error_torque[1] = error_estimate[1];
		tmp_error_torque[2] = error_estimate[2];
		tmp_error_torque[3] = error_estimate[3];

		this->outputTorqueOutputValue->setData(&tmp_error_torque);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(torque_observer);
}
;

#endif /* TORQUE_OBSERVER_HPP_ */
