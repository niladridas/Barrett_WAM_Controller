#ifndef DYNAMICS_H_
#define DYNAMICS_H_

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
class Dynamics: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Torque*/
public:
	Input<jp_type> jpInputDynamics;
	Input<jv_type> jvInputDynamics;
public:
	Output<Eigen::Matrix4d> MassMAtrixOutput;
	Output<Eigen::Vector4d> CVectorOutput;
protected:
	typename Output<Eigen::Matrix4d>::Value* MassMAtrixOutputValue;
	typename Output<Eigen::Vector4d>::Value* CVectorOutputValue;

public:
	Dynamics(/*systems::ExecutionManager* em*/) :
			jpInputDynamics(this), jvInputDynamics(this), MassMAtrixOutput(this,
					&MassMAtrixOutputValue), CVectorOutput(this,
					&CVectorOutputValue) {
//		if (em != NULL){
//		      em->startManaging(*this);
//		    }
	}

	virtual ~Dynamics() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type tmp_theta_pos;
	jv_type tmp_theta_vel;
	Eigen::Matrix4d massMatrix;
	Eigen::Vector4d Cvec;
	Eigen::Vector4d ThetaInput;
	Eigen::Vector4d ThetadotInput;

	virtual void operate() {
		tmp_theta_pos = this->jpInputDynamics.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
		massMatrix = M_4D(ThetaInput);

		tmp_theta_vel = this->jvInputDynamics.getValue();
		ThetadotInput << tmp_theta_vel[0], tmp_theta_vel[1], tmp_theta_vel[2], tmp_theta_vel[3];
		Cvec = C_4D(ThetaInput, ThetadotInput);
//		Cvec = Eigen::Vector4d::Zero(4);

		this->MassMAtrixOutputValue->setData(&massMatrix);
		this->CVectorOutputValue->setData(&Cvec);

	}

};
/* namespace Sam */
#endif /* DYNAMICS_H_ */
