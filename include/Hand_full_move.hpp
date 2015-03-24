/*
 * Hand_full.hpp
 *
 *  Created on: 21-Mar-2015
 *      Author: nilxwam
 */

#ifndef HAND_FULL_HPP_
#define HAND_FULL_HPP_

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

class Hand_full_move: public System {


	/* Torque*/
public:
	typedef Hand::jp_type hjp_t;
	Input< hjp_t > Finger_Angles;

public:
	Hand_full_move(Hand*& hand) :
			Finger_Angles(this),hand(hand){
	}

	virtual ~Hand_full_move() {
		this->mandatoryCleanUp();
	}

protected:
	hjp_t finger_angles;
	hjp_t finger_angles_current;
	Eigen::Matrix<double, 4, 1> Finger_Angles_tmp;

	Hand*& hand;

	virtual void operate() {
		finger_angles = this->Finger_Angles.getValue();

//		finger_angles[0] = Finger_Angles_tmp[0];
//		finger_angles[1] = Finger_Angles_tmp[1];
//		finger_angles[2] = Finger_Angles_tmp[2];
//		finger_angles[3] = Finger_Angles_tmp[3];


		/*
		 * Checking to see if the final configuration is possible or not
		 * If yes then the steps through which this is possible
		 */
//		finger_angles_current = hand->getInnerLinkPosition();

		// TODO
		/*
		 * CASE I:
		 * jh1,jh2,jh3 are closed(greater than Pi/2 and less than 2.4) and jh4 in any angle
		 * In this case jh4 cannot be changed
		 * jh1,jh2,jh3 has to be made say PI/2
		 *
		 */

		//hand->trapezoidalMove(finger_angles, Hand::GRASP);
//		Finger_Angles_Current_tmp[0] = finger_angles_current[0];
//		Finger_Angles_Current_tmp[1] = finger_angles_current[1];
//		Finger_Angles_Current_tmp[2] = finger_angles_current[2];
//		Finger_Angles_Current_tmp[3] = finger_angles_current[3];

	}

};

#endif /* HAND_FULL_HPP_ */
