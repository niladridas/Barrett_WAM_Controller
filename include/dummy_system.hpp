/*
 * dummy_system.hpp
 *
 *  Created on: 06-Feb-2015
 *      Author: nilxwam
 */

#ifndef DUMMY_SYSTEM_HPP_
#define DUMMY_SYSTEM_HPP_

#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>

using namespace barrett;

template<size_t DOF>
class Dummy: public System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jt_type> ControllerInput;
	Input<jt_type> CompensatorInput;
public:
	Output<jt_type> Total_torque;

protected:
	typename Output<jt_type>::Value* Total_torqueOutputValue;

public:
	Dummy(const std::string& sysName = "Dummy") :
			System(sysName), ControllerInput(this), CompensatorInput(this),Total_torque(this, &Total_torqueOutputValue) {

	}
	virtual ~Dummy() {
		this->mandatoryCleanUp();
	}

protected:
	jt_type ipT1, ipT2 ,TipT;

	virtual void operate() {
		ipT1 = this->ControllerInput.getValue();
		ipT2 = this->CompensatorInput.getValue();
		TipT = ipT1 + ipT2;
		this->Total_torqueOutputValue->setData(&TipT);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Dummy);
};

#endif /* DUMMY_SYSTEM_HPP_ */
