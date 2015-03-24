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
	Output<cf_type> Force_hand_cf;
//	Output< Eigen::Matrix<double, 3, 1> > Torque_hand; // Object Geometry
//	Output< Eigen::Matrix<double, 3, 1> > Acceleration_hand; // Object Geometry

protected:
	typename Output<Eigen::Vector3d>::Value* Force_hand_OutputValue;
	typename Output<cf_type>::Value* Force_hand_cf_OutputValue;
//	typename Output< Eigen::Matrix<double, 3, 1> >::Value* Torque_hand_OutputValue;
//	typename Output< Eigen::Matrix<double, 3, 1> >::Value* Acceleration_hand_OutputValue;

public:
	Hand_forcetorque_sense(Hand*& hand, ForceTorqueSensor*& fts) :
			Force_hand(this, &Force_hand_OutputValue), Force_hand_cf(this,
					&Force_hand_cf_OutputValue), hand(hand), fts(fts) {
	}

//	Hand_forcetorque_sense(Hand* hand, ForceTorqueSensor* fts) :
//			Force_hand(this, &Force_hand_OutputValue), Torque_hand(this,
//					&Torque_hand_OutputValue), Acceleration_hand(this,
//					&Acceleration_hand_OutputValue), hand(hand), fts(fts) {
//	}

	virtual ~Hand_forcetorque_sense() {
		this->mandatoryCleanUp();
	}

protected:

	int i;
	cf_type cf_force;
//	ct_type ct;
//	ca_type ca;
	Eigen::Vector3d Force_hand_tmp;
//	Eigen::Matrix<double, 3, 1> Torque_hand_tmp;
//	Eigen::Matrix<double, 3, 1> Acceleration_hand_tmp;

	Hand*& hand;
	ForceTorqueSensor*& fts;
	virtual void operate() {

		fts->update();
//		cf = math::saturate(fts->getForce(), 99.99);
		cf_force = fts->getForce();
//		ct = math::saturate(fts->getTorque(), 9.999);
//		ca = math::saturate(fts->getAccel(), 99.99);

//		for (i = 0; i < 3; i++) {
//			Force_hand_tmp(i, 0) = cf[i];
////			Torque_hand_tmp(i, 1) = ct[i];
////			Acceleration_hand_tmp(i, 1) = ca[i];
//		}

		Force_hand_tmp[0] = cf_force[0];
		Force_hand_tmp[1] = cf_force[1];
		Force_hand_tmp[2] = cf_force[2];

		this->Force_hand_OutputValue->setData(&Force_hand_tmp);
		this->Force_hand_cf_OutputValue->setData(&cf_force);
//		this->Torque_hand_OutputValue->setData(&Torque_hand_tmp);
//		this->Acceleration_hand_OutputValue->setData(&Acceleration_hand_tmp);

	}

};

#endif /* HAND_FORCETORQUE_SENSE_HPP_ */
