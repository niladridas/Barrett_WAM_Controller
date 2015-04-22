/*
 * tool_details.hpp
 *
 *  Created on: 27-Mar-2015
 *      Author: nilxwam
 */

#ifndef TOOL_DETAILS_HPP_
#define TOOL_DETAILS_HPP_

#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>

using namespace barrett;
using namespace systems;

/*This specific example will give sinusoidal input to the second joint, offset is given in degrees*/

template<size_t DOF>
class tool_details: public System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// IO
public:
//	Input<double> timef;
//	Output<quad_t> Quaternion_value; // TODO : Not implemented
	Output<cp_type> Cartesian_pos_value;
	Output<cv_type> Cartesian_vel_value;

protected:
//	typename System::Output<quad_t>::Value* Quaternion_valueOpValue;
	typename System::Output<cp_type>::Value* Cartesian_pos_valueOpValue;
	typename System::Output<cv_type>::Value* Cartesian_vel_valueOpValue;

public:
	tool_details(systems::Wam<DOF>& wam,const std::string& sysName = "J_ref") :
			System(sysName), wam(wam),Cartesian_pos_value(this,
					&Cartesian_pos_valueOpValue), Cartesian_vel_value(this,
					&Cartesian_vel_valueOpValue) {
	}
	virtual ~tool_details() {
		this->mandatoryCleanUp();
	}

protected:
	cp_type tool_pos;
	cv_type tool_vel;
	systems::Wam<DOF>& wam;

	virtual void operate() {
		cp_type(0.0);
		cv_type(0.0);
		tool_pos = wam.getToolPosition();
		tool_vel = wam.getToolVelocity();

		this->Cartesian_pos_valueOpValue->setData(&tool_pos);
		this->Cartesian_vel_valueOpValue->setData(&tool_vel);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(tool_details);
};

#endif /* TOOL_DETAILS_HPP_ */
