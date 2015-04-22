/*
 * Hand_forcetorque_sense.hpp
 *
 *  Created on: 23-Mar-2015
 *      Author: nilxwam
 */

#ifndef HAND_FORCETORQUE_SENSE_HPP_
#define HAND_FORCETORQUE_SENSE_HPP_

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>

#include <Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
#include <barrett/products/product_manager.h>

using namespace barrett;
using namespace systems;

template<size_t DOF>
class Hand_forcetorque_sense: public System {
	/* Torque*/
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	typedef Hand::jp_type hjp_t;

public:
	Output<Eigen::Vector3d> Force_hand; // Object Geometry

protected:
	typename Output<Eigen::Vector3d>::Value* Force_hand_OutputValue;

public:
	Hand_forcetorque_sense(Hand*& hand, ForceTorqueSensor*& fts) :
			Force_hand(this, &Force_hand_OutputValue), hand(hand), fts(fts) {
	}

	virtual ~Hand_forcetorque_sense() {
		this->mandatoryCleanUp();
	}

protected:

	int i;
	cf_type cf_force;
	Eigen::Vector3d Force_hand_tmp;

	Hand*& hand;
	ForceTorqueSensor*& fts;

	virtual void operate() {

		fts->update();
//		cf = math::saturate(fts->getForce(), 99.99);
		cf_force = fts->getForce();


		Force_hand_tmp[0] = cf_force[0];
		Force_hand_tmp[1] = cf_force[1];
		Force_hand_tmp[2] = cf_force[2];

		this->Force_hand_OutputValue->setData(&Force_hand_tmp);

	}

};

#endif /* HAND_FORCETORQUE_SENSE_HPP_ */
