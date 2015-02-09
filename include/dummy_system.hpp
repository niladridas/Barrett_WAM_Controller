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
class Dummy: public systems::SingleIO<typename units::JointTorques<DOF>::type,
		typename units::JointTorques<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Dummy(systems::ExecutionManager* em,const std::string& sysName = "Dummy") :
			systems::SingleIO<jt_type, jt_type>(sysName){
		if (em != NULL){
				      em->startManaging(*this);
				    }
	}
	virtual ~Dummy() {
		this->mandatoryCleanUp();
	}

protected:
	jt_type ipT;

	virtual void operate() {
		ipT = this->input.getValue();

		this->outputValue->setData(&ipT);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(Dummy);
};

#endif /* DUMMY_SYSTEM_HPP_ */
