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
//#include <Slidingmode_4DOF.hpp>
#include <barrett/systems/summer.h>
#include <cmath>
#include <barrett/systems/abstract/system.h>
//#include <Slidingmode_4DOF.hpp>


#include <barrett/math/traits.h>
#include <list>
#include <barrett/detail/ca_macro.h>
#include <Dynamics.h>

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
class Slidingmode_Controller: public System , Dynamics {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO
public:
	Input<jp_type> referencejpInput;
public:
	Input<jv_type> referencejvInput;
public:
	Input<ja_type> referencejaInput;

public:
	Input<jp_type> feedbackjpInput;
public:
	Input<jv_type> feedbackjvInput;

public:
	Output<jt_type> controlOutput;

protected:
	typename Output<jt_type>::Value* controlOutputValue;

public:
	Slidingmode_Controller(const Eigen::Matrix4d lamda,
			const double coeff, const double delta, const double gravity,const std::string& sysName =
					"Slidingmode_Controller") :
			System(sysName), referencejpInput(this), referencejvInput(this), referencejaInput(this), feedbackjpInput(
					this), feedbackjvInput(this), Lamda(lamda), Coeff(coeff), Delta(delta), Gravity(gravity),

			controlOutput(this, &controlOutputValue) {
	}
	virtual ~Slidingmode_Controller() {
		mandatoryCleanUp();
	}

protected:
	double Coeff, Delta, Gravity;
	jt_type jt_out;
	jp_type jp_sys, jp_ref, ep;
	jv_type jv_sys, jv_ref, ev;
	ja_type ja_ref;
	Eigen::Matrix4d K, Lamda, M;
	Eigen::Vector4d S, tmp_p, tmp_v, tmp_ev, tmp_ep, C, G, tmp_control, tmp_aref, tmp_pref, tmp_vref, jt_out_tmp;


	virtual void operate() {
		jp_ref = this->referencejpInput.getValue();
		jv_ref = this->referencejvInput.getValue();
		ja_ref = this->referencejaInput.getValue();

		jp_sys = this->feedbackjpInput.getValue();
		jv_sys = this->feedbackjvInput.getValue();
		ep = jp_sys - jp_ref;
		ev = jv_sys - jv_ref;
		K = Coeff * Lamda;

		tmp_p << jp_sys[0], jp_sys[1], jp_sys[2], jp_sys[3];
		tmp_v << jv_sys[0], jv_sys[1], jv_sys[2], jv_sys[3];
		tmp_aref << ja_ref[0], ja_ref[1], ja_ref[2], ja_ref[3];
		tmp_vref << jv_ref[0], jv_ref[1], jv_ref[2], jv_ref[3];
		tmp_pref << jp_ref[0], jp_ref[1], jp_ref[2], jp_ref[3];
		tmp_ev << ev[0], ev[1], ev[2], ev[3];
		tmp_ep << ep[0], ep[1], ep[2], ep[3];



		S = tmp_ev + Lamda * tmp_ep;

		Dynamics::makeCvec(tmp_p,
				tmp_v);
		C = Dynamics::getCvec();
		Dynamics::makeGx(tmp_p);
		G = Gravity*Dynamics::getGx();
		Dynamics::makemassMat(tmp_p);
		M = Dynamics::getmassMat();


		tmp_control[0] = S[0]/(fabs(S[0])+ Delta);
		tmp_control[1] = S[1]/(fabs(S[1])+ Delta);
		tmp_control[2] = S[2]/(fabs(S[2])+ Delta);
		tmp_control[3] = S[3]/(fabs(S[3])+ Delta);

		jt_out_tmp = C + G + M*(tmp_aref - Lamda*(tmp_v - tmp_vref) - K*tmp_control);
		jt_out[0] = jt_out_tmp[0];
		jt_out[1] = jt_out_tmp[1];
		jt_out[2] = jt_out_tmp[2];
		jt_out[3] =jt_out_tmp[3];


		this->controlOutputValue->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Slidingmode_Controller);
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
//
	std::string Key1;
	printf("Press [Enter] to turn on torque control to joint 2.");
	std::cin >> Key1;
//	waitForEnter();

	systems::Ramp time(pm.getExecutionManager(), 1.0);




//	const size_t PERIOD_MULTIPLIER = 1;
//
	J_ref<DOF> joint_ref(JT_AMPLITUDE, OMEGA, OFFSET); /*Offset is in degrees*/
	Slidingmode_Controller<DOF> slide(lamda, coeff, delta, gravity);

	systems::connect(time.output, joint_ref.timef);
	systems::connect(joint_ref.referencejpTrack, slide.referencejpInput);
	systems::connect(joint_ref.referencejvTrack, slide.referencejvInput);
	systems::connect(joint_ref.referencejaTrack, slide.referencejaInput);
	systems::connect(wam.jpOutput, slide.feedbackjpInput);
	systems::connect(wam.jvOutput, slide.feedbackjvInput);
//
	wam.trackReferenceSignal(slide.controlOutput);
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






