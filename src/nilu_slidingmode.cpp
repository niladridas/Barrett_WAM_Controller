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
	Input<const double> timef;
public:
	Output<jp_type> referencejpTrack;
public:
	Output<jv_type> referencejvTrack;
public:
	Output<ja_type> referencejaTrack;

public:
	J_ref(double amplitude, double omega, double offset,
			const std::string& sysName = "J_ref") :
			System(sysName), amp(amplitude), omega(omega), offset(offset) {
	}
	virtual ~J2control() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type jp;
	jv_type jv;
	ja_type ja;

	double amp, omega, offset;
	double theta, phi;

	virtual void operate() {
		jp(0.0);
		jv(0.0);
		ja(0.0);

		theta = omega * timef.getValue();
		phi = std::asin(3.14 * offset / 180);

		jp[1] = amp * std::sin(theta + phi);
		jv[1] = amp * omega * std::sin(theta + phi);
		ja[1] = amp * omega * omega * std::sin(theta + phi);

		referencejpTrack->setData(&jp);
		referencejvTrack->setData(&jv);
		referencejaTrack->setData(&ja);
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
	const double JT_AMPLITUDE, OMEGA, OFFSET;
	double amplitude, omega, offset;
	const Eigen::Matrix4d lamda;
	const double coeff;
	const double delta;
	const double gravity;

	lamda << 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20;
	coeff = 0.1;
	delta = 0.01;
	gravity = 0;

	jp_type startpos(0.0);
	startpos[3] = +3.14;

	std::cout
			<< "Enter the amplitude of the sinusoid for the joint position of J2: "
			<< std::endl;
	std::cin >> amplitude;

	std::cout
			<< "Enter the omega of the sinusoid for the joint position of J2: "
			<< std::endl;
	std::cin >> omega;

	std::cout
			<< "Enter the offset of the sinusoid for the joint position of J2 in degrees: "
			<< std::endl;
	std::cin >> offset;

	JT_AMPLITUDE = amplitude;
	OMEGA = omega;
	OFFSET = offset;

	printf("Press [Enter] to turn on torque control to go to zero position");
	wam.moveTo(startpos);

	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();

	systems::Ramp time(pm.getExecutionManager(), 1.0);

	const size_t PERIOD_MULTIPLIER = 1;

	J_ref<DOF> joint_ref(JT_AMPLITUDE, OMEGA, OFFSET); /*Offset is in degrees*/
	Slidingmode_Controller<DOF> slide(lamda, coeff, delta, gravity);

	systems::connect(time.output, joint_ref.timef);
	systems::connect(joint_ref.referencejpTrack,
			slide.referencejpInput);
	systems::connect(joint_ref.referencejvTrack,
			slide.referencejvInput);
	systems::connect(joint_ref.referencejaTrack,
			slide.referencejaInput);
	systems::connect(wam.jpOutput, slide.feedbackjpInput);
	systems::connect(wam.jvOutput, slide.feedbackjvInput);

	wam.trackReferenceSignal(slide.controlOutput);
	time.smoothStart(TRANSITION_DURATION);

	printf("Press [Enter] to stop.");
	waitForEnter();

	time.smoothStop(TRANSITION_DURATION);
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}

