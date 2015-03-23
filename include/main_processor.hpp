/*
 * main_processor.hpp
 *
 *  Created on: 23-Mar-2015
 *      Author: nilxwam
 */

#ifndef MAIN_PROCESSOR_HPP_
#define MAIN_PROCESSOR_HPP_

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
class main_processor: public System {
	/* Torque*/
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	typedef Hand::jp_type hjp_t;
	Input<double> Current_time;
	Input<Eigen::Matrix<double, 3, 1>> Force_hand; //
	Input<Eigen::Matrix<double, 3, 1>> Torque_hand; //
	Input<Eigen::Matrix<double, 3, 1>> Acceleration_hand; //
	Input<Eigen::Matrix<double, 8, 3>> Finger_Tactile_1; // Object Geometry
	Input<Eigen::Matrix<double, 8, 3>> Finger_Tactile_2; // Object Geometry
	Input<Eigen::Matrix<double, 8, 3>> Finger_Tactile_3; // Object Geometry
	Input<Eigen::Matrix<double, 8, 3>> Finger_Tactile_4; // Object Geometry

public:
	Output<Eigen::Matrix<double, 4, 1>> Desired_Finger_Angles; // To move the hand
//	Output<Eigen::Matrix<double, 3, 1>> Torque_hand; // Some other descriptions to move the entire arm

protected:
	typename Output<Eigen::Matrix<double, 4, 1>>::Value* Desired_Finger_Angles_OutputValue;
//	typename Output<Eigen::Matrix<double, 3, 1>>::Value* Torque_hand_OutputValue;
//	typename Output<Eigen::Matrix<double, 3, 1>>::Value* Acceleration_hand_OutputValue;

public:
	main_processor(Hand* hand, double delta_step, double spread_angle,
			double threshold_impulse, bool Release_Mode) :
			Current_time(this), Force_hand(this), Torque_hand(this), Acceleration_hand(
					this), Finger_Tactile_1(this), Finger_Tactile_2(this), Finger_Tactile_3(
					this), Finger_Tactile_4(this), Desired_Finger_Angles(this,
					&Desired_Finger_Angles_OutputValue), hand(hand), delta_step(
					delta_step), spread_angle(spread_angle), threshold_impulse(
					threshold_impulse), Release_Mode(Release_Mode) {
		Force_hand_tmp_previous << 0, 0, 0;
		Previous_time = 0;
	}

	virtual ~main_processor() {
		this->mandatoryCleanUp();
	}

protected:
	double delta_step, delta_step1, delta_step2, delta_step3, delta_step4;
	int i, j;
	int value1, value2, value3, value4;
	double spread_angle;
	Eigen::Matrix<double, 3, 1> Force_hand_tmp;
	Eigen::Matrix<double, 3, 1> Torque_hand_tmp;
	Eigen::Matrix<double, 3, 1> Acceleration_hand_tmp;
	double Previous_time;
	double Current_time_tmp;
	hjp_t finger_angles_current;
	Eigen::Matrix<double, 4, 1> Finger_Angles_Current;
	Eigen::Matrix<double, 4, 1> Finger_Angles_Updated;
	Hand* hand;
	Eigen::Matrix<double, 8, 3> Finger_Tactile_1_tmp; // Object Geometry
	Eigen::Matrix<double, 8, 3> Finger_Tactile_2_tmp; // Object Geometry
	Eigen::Matrix<double, 8, 3> Finger_Tactile_3_tmp; // Object Geometry
	Eigen::Matrix<double, 8, 3> Finger_Tactile_4_tmp; // Object Geometry

	Eigen::Matrix<double, 3, 1> Force_hand_tmp; //
	Eigen::Matrix<double, 3, 1> Torque_hand_tmp; //
	Eigen::Matrix<double, 3, 1> Acceleration_hand_tmp; //
	Eigen::Matrix<double, 3, 1> Force_hand_tmp;
	Eigen::Matrix<double, 3, 1> Force_hand_tmp_previous;
	double threshold_impulse;
	bool Release_Mode;

	virtual void operate() {

		Current_time_tmp = this->Current_time;
		hand->update();
		finger_angles_current = hand->getInnerLinkPosition();
		Finger_Angles_Current[0] = finger_angles_current[0];
		Finger_Angles_Current[1] = finger_angles_current[1];
		Finger_Angles_Current[2] = finger_angles_current[2];
		Finger_Angles_Current[3] = finger_angles_current[3];

		Finger_Tactile_1_tmp = this->Finger_Tactile_1.getValue();
		Finger_Tactile_2_tmp = this->Finger_Tactile_2.getValue();
		Finger_Tactile_3_tmp = this->Finger_Tactile_3.getValue();
		Finger_Tactile_4_tmp = this->Finger_Tactile_4.getValue();

		Force_hand_tmp = this->Force_hand.getValue();
		Torque_hand_tmp = this->Torque_hand.getValue();
		Acceleration_hand_tmp = this->Acceleration_hand.getValue();

		if ((Force_hand_tmp_previous - Force_hand_tmp).Eigen::norm()
				/ (Current_time_tmp - Previous_time) <= threshold_impulse
				&& Release_Mode == 0) {

			/*
			 * Check if any pressure sensor exceeds the pressure limit
			 */
//	Check for the first finger F1
			for (i = 0; i < 8; i++) {
				for (j = 0; j < 3; j++) {
					value1 = (int) (Finger_Tactile_1_tmp(i, j) * 256.0) / 102; // integer division
					if (value1 >= 7) {
						delta_step1 = 0;
						break;
					} else
						delta_step1 = delta_step;
				}
			}
			//	Check for the first finger F2

			for (i = 0; i < 8; i++) {
				for (j = 0; j < 3; j++) {
					value2 = (int) (Finger_Tactile_2_tmp(i, j) * 256.0) / 102; // integer division
					if (value1 >= 7) {
						delta_step2 = 0;
						break;
					} else
						delta_step2 = delta_step;
				}
			}
			//	Check for the first finger F3

			for (i = 0; i < 8; i++) {
				for (j = 0; j < 3; j++) {
					value4 = (int) (Finger_Tactile_4_tmp(i, j) * 256.0) / 102; // integer division
					if (value1 >= 7) {
						delta_step3 = 0;
						break;
					} else
						delta_step3 = delta_step;
				}
			}

			/*
			 * Check for the maximum angles
			 */
			if (Finger_Angles_Current[0] < 2.14) {
				Finger_Angles_Updated[0] = Finger_Angles_Current[0]
						+ delta_step1 * 0.0016;
			} else
				Finger_Angles_Updated[0] = 2.14;

			if (Finger_Angles_Current[1] < 2.14) {
				Finger_Angles_Updated[1] = Finger_Angles_Current[1]
						+ delta_step2 * 0.0016;
			} else
				Finger_Angles_Updated[1] = 2.14;

			if (Finger_Angles_Current[2] < 2.14) {
				Finger_Angles_Updated[2] = Finger_Angles_Current[2]
						+ delta_step3 * 0.0016;
			} else
				Finger_Angles_Updated[2] = 2.14;

			Finger_Angles_Updated[3] = spread_angle;

		} else {
			Release_Mode = 1;

			if (Finger_Angles_Current[0] > 0) {
				Finger_Angles_Updated[0] = Finger_Angles_Current[0]
						- 0.002 * 0.0016;
			} else
				Finger_Angles_Updated[0] = 0;

			if (Finger_Angles_Current[1] > 0) {
				Finger_Angles_Updated[1] = Finger_Angles_Current[1]
						- 0.002 * 0.0016;
			} else
				Finger_Angles_Updated[1] = 0;

			if (Finger_Angles_Current[2] > 0) {
				Finger_Angles_Updated[2] = Finger_Angles_Current[2]
						- 0.002 * 0.0016;
			} else
				Finger_Angles_Updated[2] = 0;

			Finger_Angles_Updated[3] = spread_angle;

		}

		this->Desired_Finger_Angles_OutputValue->setData(
				&Finger_Angles_Updated);

		Force_hand_tmp_previous = Force_hand_tmp;
		Previous_time = Current_time_tmp;
	}

};

#endif /* MAIN_PROCESSOR_HPP_ */
