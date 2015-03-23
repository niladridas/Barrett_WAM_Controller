/*
 * Hand_tactile_sense.hpp
 *
 *  Created on: 22-Mar-2015
 *      Author: nilxwam
 */

#ifndef HAND_TACTILE_SENSE_HPP_
#define HAND_TACTILE_SENSE_HPP_

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

class Hand_tactile_sense: public System {
	/* Torque*/
public:
	typedef Hand::jp_type hjp_t;

public:
	Output<Eigen::Matrix<double, 8, 3>> Finger_Tactile_1; // Object Geometry
	Output<Eigen::Matrix<double, 8, 3>> Finger_Tactile_2; // Object Geometry
	Output<Eigen::Matrix<double, 8, 3>> Finger_Tactile_3; // Object Geometry
	Output<Eigen::Matrix<double, 8, 3>> Finger_Tactile_4; // Object Geometry

protected:
	typename Output<Eigen::Matrix<double, 8, 3>>::Value* Finger_Tactile_1_OutputValue;
	typename Output<Eigen::Matrix<double, 8, 3>>::Value* Finger_Tactile_2_OutputValue;
	typename Output<Eigen::Matrix<double, 8, 3>>::Value* Finger_Tactile_3_OutputValue;
	typename Output<Eigen::Matrix<double, 8, 3>>::Value* Finger_Tactile_4_OutputValue;

public:
	Hand_tactile_sense(Hand* hand, std::vector<TactilePuck*> tps) :
			Finger_Tactile_1(this, &Finger_Tactile_1_OutputValue), Finger_Tactile_2(
					this, &Finger_Tactile_2_OutputValue), Finger_Tactile_3(this,
					&Finger_Tactile_3_OutputValue), Finger_Tactile_4(this,
					&Finger_Tactile_4_OutputValue), hand(hand), tps(tps) {
	}

	virtual ~Hand_tactile_sense() {
		this->mandatoryCleanUp();
	}

protected:
	hjp_t finger_angles_current;
	Eigen::Matrix<double, 4, 1> Finger_Angles_Current_tmp;
	TactilePuck::v_type F1t; // Vector of size 24
	TactilePuck::v_type F2t;
	TactilePuck::v_type F3t;
	TactilePuck::v_type F4t;
	Eigen::Matrix<double, 8, 3> F1_tact;
	Eigen::Matrix<double, 8, 3> F2_tact;
	Eigen::Matrix<double, 8, 3> F3_tact;
	Eigen::Matrix<double, 8, 3> F4_tact;
	int i, j;

	Hand* hand;
	std::vector<TactilePuck*> tps;
	virtual void operate() {

		hand->update();

		finger_angles_current = hand->getInnerLinkPosition();
		Finger_Angles_Current_tmp[0] = finger_angles_current[0];
		Finger_Angles_Current_tmp[1] = finger_angles_current[1];
		Finger_Angles_Current_tmp[2] = finger_angles_current[2];
		Finger_Angles_Current_tmp[3] = finger_angles_current[3];

		F1t = tps[0]->getFullData();
		F2t = tps[1]->getFullData();
		F3t = tps[2]->getFullData();
		F4t = tps[3]->getFullData();

		for (i = 0; i < 8; i++) {
			for (j = 0; j < 3; j++) {
				F1_tact(i, j) = F1t[3 * i + j];
				F2_tact(i, j) = F2t[3 * i + j];
				F3_tact(i, j) = F3t[3 * i + j];
				F4_tact(i, j) = F4t[3 * i + j];
			}
		}

		this->Finger_Tactile_1_OutputValue->setData(
				&F1_tact);
		this->Finger_Tactile_2_OutputValue->setData(
						&F2_tact);
		this->Finger_Tactile_3_OutputValue->setData(
						&F3_tact);
		this->Finger_Tactile_4_OutputValue->setData(
						&F4_tact);

	}

};

#endif /* HAND_TACTILE_SENSE_HPP_ */
