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

#include <eigen3/Eigen/Core>
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
	Output<Eigen::Matrix<double, 3, 1>> Force_hand; // Object Geometry
	Output<Eigen::Matrix<double, 3, 1>> Torque_hand; // Object Geometry
	Output<Eigen::Matrix<double, 3, 1>> Acceleration_hand; // Object Geometry

protected:
	typename Output<Eigen::Matrix<double, 3, 1>>::Value* Force_hand_OutputValue;
	typename Output<Eigen::Matrix<double, 3, 1>>::Value* Torque_hand_OutputValue;
	typename Output<Eigen::Matrix<double, 3, 1>>::Value* Acceleration_hand_OutputValue;

public:
	Hand_forcetorque_sense(Hand* hand, ForceTorqueSensor* fts) :
			Force_hand(this, &Force_hand_OutputValue), Torque_hand(this,
					&Torque_hand_OutputValue), Acceleration_hand(this,
					&Acceleration_hand_OutputValue), hand(hand), fts(fts) {
	}

	virtual ~Hand_forcetorque_sense() {
		this->mandatoryCleanUp();
	}

protected:

	int i;
	cf_type cf;
	ct_type ct;
	ca_type ca;
	Eigen::Matrix<double, 3, 1> Force_hand_tmp;
	Eigen::Matrix<double, 3, 1> Torque_hand_tmp;
	Eigen::Matrix<double, 3, 1> Acceleration_hand_tmp;

	Hand* hand;
	ForceTorqueSensor* fts;
	virtual void operate() {

		hand->update();
		cf = math::saturate(fts->getForce(), 99.99);
		ct = math::saturate(fts->getTorque(), 9.999);
		ca = math::saturate(fts->getAccel(), 99.99);

		for(i=0;i<3;i++)
		{
			Force_hand_tmp(i,1) = cf[i];
			Torque_hand_tmp(i,1) = ct[i];
			Acceleration_hand_tmp(i,1) = ca[i];
		}

		this->Force_hand_OutputValue->setData(&Force_hand_tmp);
		this->Torque_hand_OutputValue->setData(&Torque_hand_tmp);
		this->Acceleration_hand_OutputValue->setData(&Acceleration_hand_tmp);

	}

};

#endif /* HAND_FORCETORQUE_SENSE_HPP_ */
