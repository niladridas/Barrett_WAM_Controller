/*
 * Dynamics_mod.hpp
 *
 *  Created on: 25-Feb-2015
 *      Author: nilxwam
 */

#ifndef DYNAMICS_MOD_HPP_
#define DYNAMICS_MOD_HPP_

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

//#include <barrett/detail/ca_macro.h>
//#include <barrett/math/traits.h>
//#include <barrett/systems/abstract/execution_manager.h>
//#include <barrett/systems/abstract/controller.h>
//#include <samlibs.h>
//#include <tsk.h>

#include <M_4D.hpp>
#include <C_4D.hpp>

using namespace barrett;
using namespace systems;

template<size_t DOF>
class Dynamics_mod: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Torque*/
public:
	Input<jp_type> referencejpInput; // qd
	Input<jv_type> referencejvInput; // qdd
	Input<jp_type> feedbackjpInput;
	Input<jv_type> feedbackjvInput;
public:
	Output<Eigen::Matrix4d> MassMAtrixOutput;
	Output<Eigen::Vector4d> CVectorOutput;
	Output<Eigen::Matrix4d> MassMAtrixOutputref;
	Output<Eigen::Vector4d> CVectorOutputref;
protected:
	typename Output<Eigen::Matrix4d>::Value* MassMAtrixOutputValue;
	typename Output<Eigen::Vector4d>::Value* CVectorOutputValue;
	typename Output<Eigen::Matrix4d>::Value* MassMAtrixOutputValueref;
	typename Output<Eigen::Vector4d>::Value* CVectorOutputValueref;

public:
	Dynamics_mod(/*systems::ExecutionManager* em*/) :
			referencejpInput(this), referencejvInput(this), feedbackjpInput(
					this), feedbackjvInput(this), MassMAtrixOutput(this,
					&MassMAtrixOutputValue), CVectorOutput(this,
					&CVectorOutputValue), MassMAtrixOutputref(this,
					&MassMAtrixOutputValueref), CVectorOutputref(this,
					&CVectorOutputValueref) {

	}

	virtual ~Dynamics_mod() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type tmp_theta_pos;
	jv_type tmp_theta_vel;
	jp_type tmp_theta_posref;
	jv_type tmp_theta_velref;
	Eigen::Matrix4d massMatrix;
	Eigen::Vector4d Cvec;
	Eigen::Matrix4d massMatrixref;
	Eigen::Vector4d Cvecref;
	Eigen::Vector4d ThetaInput;
	Eigen::Vector4d ThetadotInput;
	Eigen::Vector4d ThetaInputref;
	Eigen::Vector4d ThetadotInputref;

	virtual void operate() {
		tmp_theta_pos = this->referencejpInput.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
		massMatrix = M_4D(ThetaInput);

		tmp_theta_vel = this->referencejvInput.getValue();
		ThetadotInput << tmp_theta_vel[0], tmp_theta_vel[1], tmp_theta_vel[2], tmp_theta_vel[3];
		Cvec = C_4D(ThetaInput, ThetadotInput);

		tmp_theta_posref = this->feedbackjpInput.getValue();
		ThetaInputref << tmp_theta_posref[0], tmp_theta_posref[1], tmp_theta_posref[2], tmp_theta_posref[3];
		massMatrixref = M_4D(ThetaInputref);

		tmp_theta_velref = this->feedbackjvInput.getValue();
		ThetadotInputref << tmp_theta_velref[0], tmp_theta_velref[1], tmp_theta_velref[2], tmp_theta_velref[3];
		Cvecref = C_4D(ThetaInputref, ThetadotInputref);

		this->MassMAtrixOutputValue->setData(&massMatrix);
		this->CVectorOutputValue->setData(&Cvec);
		this->MassMAtrixOutputValueref->setData(&massMatrixref);
		this->CVectorOutputValueref->setData(&Cvecref);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(Dynamics_mod);
}
;
#endif /* DYNAMICS_MOD_HPP_ */
