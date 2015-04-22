/*
 * reference_signal.hpp
 *
 *  Created on: 02-Feb-2015
 *      Author: nilxwam
 */

#ifndef REFERENCE_SIGNAL_HPP_
#define REFERENCE_SIGNAL_HPP_

#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>

//
//#include <barrett/math/traits.h>
//#include <list>
//#include <barrett/units.h>
//#include <barrett/detail/ca_macro.h>
//#include <barrett/systems/abstract/system.h>
//
//#include <eigen3/Eigen/Core>
//#include <libconfig.h++>
//
//#include <barrett/detail/ca_macro.h>
//#include <barrett/math/traits.h>
//#include <barrett/systems/abstract/execution_manager.h>
//#include <barrett/systems/abstract/controller.h>
//#include <samlibs.h>

using namespace barrett;
using namespace systems;

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
	J_ref(/*systems::ExecutionManager* em,*/const Eigen::Vector4d amplitude,
			const Eigen::Vector4d omega, const jp_type startpos,
			const std::string& sysName = "J_ref") :
			System(sysName), timef(this), referencejpTrack(this,
					&referencejpOpValue), referencejvTrack(this,
					&referencejvOpValue), referencejaTrack(this,
					&referencejaOpValue), amplitude(amplitude), omega(omega), startpos(
					startpos) {
//		if (em != NULL){
//		      em->startManaging(*this);
//		    }
	}
	virtual ~J_ref() {
		this->mandatoryCleanUp();
	}

protected:
	Eigen::Vector4d amplitude;
	Eigen::Vector4d omega;
	Eigen::Vector4d theta;
	jp_type startpos;
	jp_type jp;
	jv_type jv;
	ja_type ja;

	virtual void operate() {
		jp(0.0);
		jv(0.0);
		ja(0.0);

		theta[0] = omega[0] * this->timef.getValue();
		theta[1] = omega[1] * this->timef.getValue();
		theta[2] = omega[2] * this->timef.getValue();
		theta[3] = omega[3] * this->timef.getValue();


//		jp[0] = startpos[0];
//		jp[1] = startpos[1];
//		jp[2] = amplitude * std::sin(theta) + startpos[2];
//		jp[3] = amplitude * std::cos(theta) + startpos[3] ;//- 1.0) + startpos[3];
//
//		jv[2] = amplitude * omega * std::cos(theta);
//		jv[3] = - amplitude * omega * std::sin(theta);
//
//		ja[2] = - amplitude * omega *omega * std::sin(theta);
//		ja[3] = - amplitude * omega *omega * std::cos(theta);

//		jp[0] = amplitude * std::sin(theta);
//		jv[0] = amplitude * omega * std::cos(theta);
//		ja[0] = -amplitude * omega * omega * std::sin(theta);
//
//		jp[1] = -1.047 + (0.5236) * std::sin(theta);
//		jv[1] = (0.5236) * omega * std::cos(theta);
//		ja[1] = -(0.5236) * omega * omega * std::sin(theta);
//
//		jp[2] = (amplitude / 2) * std::sin(theta);
//		jv[2] = (amplitude / 2) * omega * std::cos(theta);
//		ja[2] = -(amplitude / 2) * omega * omega * std::sin(theta);
//
//		jp[3] = (M_PI / 2) + amplitude * std::sin(theta); //+3.14;
//		jv[3] = amplitude * omega * std::cos(theta);
//		ja[3] = -amplitude * omega * omega * std::sin(theta);



		jp[0] = startpos[0] + amplitude[0] * std::sin(theta[0]);
		jv[0] =  amplitude[0] * omega[0] * std::cos(theta[0]);
		ja[0] = -amplitude[0] * omega[0] * omega[0] * std::sin(theta[0]);

		jp[1] = startpos[1] + amplitude[1]  * std::sin(theta[1]);
		jv[1] = amplitude[1]  * omega[1] * std::cos(theta[1]);
		ja[1] = -amplitude[1]  * omega[1] * omega[1] * std::sin(theta[1]);

		jp[2] = startpos[2] + amplitude[2] * std::sin(theta[2]);
		jv[2] = amplitude[2] * omega[2] * std::cos(theta[2]);
		ja[2] = -amplitude[2] * omega[2] * omega[2] * std::sin(theta[2]);

		jp[3] = startpos[3] + amplitude[3] * std::sin(theta[3]); //+3.14;
		jv[3] = amplitude[3] * omega[3] * std::cos(theta[3]);
		ja[3] = -amplitude[3] * omega[3] * omega[3] * std::sin(theta[3]);


		this->referencejpOpValue->setData(&jp);
		this->referencejvOpValue->setData(&jv);
		this->referencejaOpValue->setData(&ja);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(J_ref);
};

#endif /* REFERENCE_SIGNAL_HPP_ */
