/*
 * disturbance_observer.hpp
 *
 *  Created on: 07-May-2015
 *      Author: nilxwam
 */

#ifndef DISTURBANCE_OBSERVER_HPP_
#define DISTURBANCE_OBSERVER_HPP_

#include <stdlib.h>
#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
//#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <samlibs.h>

using namespace barrett;
using namespace systems;
using namespace Eigen;

template<size_t DOF>
class disturbance_observer: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<double> time;
	Input<jp_type> inputactual_pos;
	Input<jv_type> inputactual_vel;
	Input<Eigen::Matrix4d> M;
	Input<Eigen::Vector4d> C;
	Input<jt_type> SMC;

public:
	Output<jt_type> disturbance_torque_etimate;
	Output<jt_type> Z_copy;
	Output<double> Test;
	Output<jp_type> InverseTest;

protected:
	typename Output<jt_type>::Value* disturbance_torque_etimateValue;
	typename Output<jt_type>::Value* Z_copyValue;
	typename Output<double>::Value* TestValue;
	typename Output<jp_type>::Value*InverseTestValue;

public:
	disturbance_observer(const double dist_K, const int state,
			const Eigen::Vector4d initial_velocity, const float A) :
			dist_K(dist_K), state(state), initial_velocity(initial_velocity), A(A),time(
					this), inputactual_pos(this), inputactual_vel(this), M(
					this), C(this), SMC(this), disturbance_torque_etimate(this,
					&disturbance_torque_etimateValue), Z_copy(this,
					&Z_copyValue), Test(this, &TestValue), InverseTest(this,
					&InverseTestValue) {
//		Z = Eigen::Vector4d::Zero();
		Z_begin = Eigen::Vector4d::Zero();
		P = Eigen::Vector4d::Zero();
		torque_estimate = Eigen::Vector4d::Zero();
		test = 0;
	}

	virtual ~disturbance_observer() {
		this->mandatoryCleanUp();
	}

protected:
	Eigen::Vector4d SMC_torque;
	Eigen::Matrix4d M_i;
	Eigen::Vector4d C_i;
	Eigen::Vector2d time_vector;
	double del_time;
	jt_type jt_torque_estimate;
	Eigen::Vector4d actual_velocity;
	Eigen::Vector4d Z;
	Eigen::Vector4d old_delta_Z;
	Eigen::Vector4d new_delta_Z;
	Eigen::Vector4d Z_end;
	Eigen::Vector4d Z_begin;
	jt_type Z_tmp;
	Eigen::Vector4d torque_estimate;
	Eigen::Vector4d P;
	Eigen::Matrix4d L;
	Eigen::Matrix4d Y;
	double dist_K;
	double test;
	Eigen::Matrix4d inverse_test;
	jp_type inverse_test_vector;
	int state;
	Eigen::Vector4d initial_velocity;
	float A;
//	Eigen::Vector4d initial_velocity_vec;

	virtual void operate() {

		time_vector[0] = time_vector[1];
		time_vector[1] = this->time.getValue();
		del_time = time_vector[1] - time_vector[0];
		actual_velocity[0] = this->inputactual_vel.getValue()[0];
		actual_velocity[1] = this->inputactual_vel.getValue()[1];
		actual_velocity[2] = this->inputactual_vel.getValue()[2];
		actual_velocity[3] = this->inputactual_vel.getValue()[3];
		M_i = this->M.getValue();
		C_i = this->C.getValue();
		SMC_torque[0] = this->SMC.getValue()[0];
		SMC_torque[1] = this->SMC.getValue()[1];
		SMC_torque[2] = this->SMC.getValue()[2];
		SMC_torque[3] = this->SMC.getValue()[3];
		Y = dist_K * Eigen::Matrix4d::Identity();
//		L = Y;
		Z_tmp[0] = Z[0];
		Z_tmp[1] = Z[1];
		Z_tmp[2] = Z[2];
		Z_tmp[3] = Z[3];
		L = Y * M_i.inverse();
		inverse_test = M_i * L;
		inverse_test_vector[0] = inverse_test(0, 0);
		inverse_test_vector[1] = inverse_test(1, 1);
		inverse_test_vector[2] = inverse_test(2, 2);
		inverse_test_vector[3] = inverse_test(3, 3);
//		initial_velocity_vec[0] = initial_velocity[0];
//		initial_velocity_vec[1] = initial_velocity[1];
//		initial_velocity_vec[2] = initial_velocity[2];
//		initial_velocity_vec[3] = initial_velocity[3];
		P = Y * (actual_velocity - A * initial_velocity);
//		P = 0*actual_velocity;

		if (state == 1) {
			old_delta_Z = (-L * Z_begin
					+ L * (C_i - (SMC_torque - torque_estimate) - P));

			Z_end = Z_begin + old_delta_Z * del_time;
			new_delta_Z = (-L * Z_end
					+ L * (C_i - (SMC_torque - torque_estimate) - P));

			Z_begin = Z_begin + 0.5 * (new_delta_Z + old_delta_Z) * del_time;

			torque_estimate = Z_begin + P;
		}

		else {
			old_delta_Z = (-L * Z_begin
					+ L * (C_i - (SMC_torque - torque_estimate) - P));
			Z_begin = Z_begin + old_delta_Z * del_time;
			torque_estimate = Z_begin + P;

		}
		test = test + del_time;
		if (std::abs(torque_estimate[0]) > 0.1)
		{
			torque_estimate[0] = 0;
		}
		if (std::abs(torque_estimate[1]) > 3) {
			torque_estimate[1] = 0;
		}
		if (std::abs(torque_estimate[2]) > 0.1)
		{
			torque_estimate[2] = 0;
		}
		if (std::abs(torque_estimate[3]) > 0.1)
		{
			torque_estimate[3] = 0;
		}
		jt_torque_estimate[0] = -torque_estimate[0];
		jt_torque_estimate[1] = -torque_estimate[1];
		jt_torque_estimate[2] = -torque_estimate[2];
		jt_torque_estimate[3] = -torque_estimate[3];
//		jt_torque_estimate[0] = 0;
////		jt_torque_estimate[1] = 0;
//		jt_torque_estimate[2] = 0;
//		jt_torque_estimate[3] = 0;
		this->disturbance_torque_etimateValue->setData(&jt_torque_estimate);
		this->Z_copyValue->setData(&Z_tmp);
		this->TestValue->setData(&test);
		this->InverseTestValue->setData(&inverse_test_vector);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(disturbance_observer);
}
;

#endif /* DISTURBANCE_OBSERVER_HPP_ */
