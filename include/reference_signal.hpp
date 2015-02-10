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
	J_ref(/*systems::ExecutionManager* em,*/double amplitude, double omega, const std::string& sysName = "J_ref") :
			System(sysName), timef(this), referencejpTrack(this,
					&referencejpOpValue), referencejvTrack(this,
					&referencejvOpValue), referencejaTrack(this,
					&referencejaOpValue), amplitude(amplitude), omega(omega) {
//		if (em != NULL){
//		      em->startManaging(*this);
//		    }
	}
	virtual ~J_ref() {
		this->mandatoryCleanUp();
	}

protected:
	double amplitude;
	double omega;
	double theta;
	jp_type jp;
	jv_type jv;
	ja_type ja;

	virtual void operate() {
		jp(0.0);
		jv(0.0);
		ja(0.0);

		theta = omega * this->timef.getValue();

		jp[1] = amplitude * std::sin(theta);
		jp[3] = +3.14;
		jv[1] = amplitude * omega * std::cos(theta);
		ja[1] = -amplitude * omega * omega * std::sin(theta);

		this->referencejpOpValue->setData(&jp);
		this->referencejvOpValue->setData(&jv);
		this->referencejaOpValue->setData(&ja);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(J_ref);
};

#endif /* REFERENCE_SIGNAL_HPP_ */
