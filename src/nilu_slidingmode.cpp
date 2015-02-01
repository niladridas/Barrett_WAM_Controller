/*
 * nilu_slidingmode_J2.cpp
 *
 *  Created on: 30-Jan-2015
 *      Author: nilxwam
 */

#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>
#include <Slidingmode_4DOF.hpp>
#include <barrett/systems/summer.h>
#include <cmath>
#include <barrett/systems/abstract/system.h>
#include <Slidingmode_4DOF.hpp>

using namespace barrett;
using namespace systems;
//using detail::waitForEnter;

/*This specific example will give sinusoidal input to the second joint, offset is given in degrees*/

template<size_t DOF>
class J_ref: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// IO
public:
	Input<double> timef;
	Output<jp_type> referencejpTrack;
	Output<jv_type> referencejvTrack;
	Output<ja_type> referencejaTrack;

protected:
	typename System::Output<jp_type>::Value* referencejpOpValue;
	typename System::Output<jv_type>::Value* referencejvOpValue;
	typename System::Output<ja_type>::Value* referencejaOpValue;

public:
	J_ref(double amplitude, double omega, double offset,
			const std::string& sysName = "J_ref") :
			 amp(amplitude), omg(omega), off(offset), System(sysName),timef(
					this), referencejpTrack(this, &referencejpOpValue), referencejvTrack(
					this, &referencejvOpValue), referencejaTrack(this,
					&referencejaOpValue) {
	}
	virtual ~J_ref() {
		this->mandatoryCleanUp();
	}

protected:
	double amp, omg, off;
	double theta, phi;
	jp_type jp;
	jv_type jv;
	ja_type ja;
	virtual void operate() {
		jp(0.0);
		jv(0.0);
		ja(0.0);

		theta = omg * timef.getValue();
		phi = std::asin(3.14 * off / 180);

		jp[1] = amp * std::sin(theta + phi);
		jv[1] = amp * omg * std::sin(theta + phi);
		ja[1] = amp * omg * omg * std::sin(theta + phi);

		referencejpOpValue->setData(&jp);
		referencejvOpValue->setData(&jv);
		referencejaOpValue->setData(&ja);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(J_ref);
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();

	const double TRANSITION_DURATION = 0.5; // seconds

	double amplitude1, omega1, offset1;
	Eigen::Matrix4d tmp_lamda;
	tmp_lamda << 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20;
	const Eigen::Matrix4d lamda = tmp_lamda;
	const double coeff = 0.1;
	const double delta = 0.01;
	const double gravity = 0;

	jp_type startpos(0.0);
	startpos[3] = +3.14;

	std::cout
			<< "Enter the amplitude of the sinusoid for the joint position of J2: "
			<< std::endl;
	std::cin >> amplitude1;

	std::cout
			<< "Enter the omega of the sinusoid for the joint position of J2: "
			<< std::endl;
	std::cin >> omega1;

	std::cout
			<< "Enter the offset of the sinusoid for the joint position of J2 in degrees: "
			<< std::endl;
	std::cin >> offset1;

	const double JT_AMPLITUDE = amplitude1;
	const double OMEGA = omega1;
	const double OFFSET = offset1;

	printf("Press [Enter] to turn on torque control to go to zero position");
	wam.moveTo(startpos);

	std::string Key1;
	printf("Press [Enter] to turn on torque control to joint 2.");
	std::cin >> Key1;
//	waitForEnter();

	systems::Ramp time(pm.getExecutionManager(), 1.0);

//	const size_t PERIOD_MULTIPLIER = 1;

	J_ref<DOF> joint_ref(JT_AMPLITUDE, OMEGA, OFFSET); /*Offset is in degrees*/
	Slidingmode_Controller<DOF> slide(lamda, coeff, delta, gravity);

//	systems::connect(time.output, joint_ref.timef);
//	systems::connect(joint_ref.referencejpTrack, slide.referencejpInput);
//	systems::connect(joint_ref.referencejvTrack, slide.referencejvInput);
//	systems::connect(joint_ref.referencejaTrack, slide.referencejaInput);
//	systems::connect(wam.jpOutput, slide.feedbackjpInput);
//	systems::connect(wam.jvOutput, slide.feedbackjvInput);
//
//	wam.trackReferenceSignal(slide.controlOutput);
	time.smoothStart(TRANSITION_DURATION);

	std::string Key2;
	printf("Press [Enter] to stop.");
//	waitForEnter();
	std::cin >> Key2;

	time.smoothStop(TRANSITION_DURATION);
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}

