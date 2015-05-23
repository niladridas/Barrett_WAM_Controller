/*
 * tactile_feedback_move.cpp
 *
 *  Created on: 18-May-2015
 *      Author: nilxwam
 */

#include <curses.h>
#include <stdlib.h>
#include <barrett/os.h>  // For btsleep()
#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

using namespace barrett;
//using detail::waitForEnter;

#include <barrett/standard_main_function.h>
//#include <Hand_forcetorque_sense.hpp>
#include <barrett/log.h>
#include <samlibs.h>

#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/detail/stl_utils.h>

void waitForEnter() {
	std::string line;
	std::getline(std::cin, line);
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	// Is a Hand attached?
	Hand* hand = NULL;
	wam.gravityCompensate();
	if (pm.foundHand()) {
		hand = pm.getHand();

		printf(
				">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		waitForEnter();
		hand->initialize();
	}
	std::vector<TactilePuck*> tps;
	tps = hand->getTactilePucks();
	TactilePuck::v_type F1t; // Vector of size 24
	TactilePuck::v_type F2t;
	TactilePuck::v_type F3t;
	TactilePuck::v_type F4t;
	Eigen::Matrix<double, 8, 3> F1_tact;
	Eigen::Matrix<double, 8, 3> F2_tact;
	Eigen::Matrix<double, 8, 3> F3_tact;
	Eigen::Matrix<double, 8, 3> F4_tact;
	int i, j;
	typedef Hand::jp_type hjp_t;
	hjp_t finger_angles;
	Eigen::VectorXd finger;
	Eigen::VectorXd finger_resolution;
	Eigen::VectorXd finger_pressure_max;
	Sam::initEigenVec<double>(finger, Sam::readFile<double>("finger.txt"));
	Sam::initEigenVec<double>(finger_resolution,
			Sam::readFile<double>("finger_resolution.txt"));
	Sam::initEigenVec<double>(finger_pressure_max,
			Sam::readFile<double>("finger_pressure_max.txt"));

	finger_angles[0] = finger[0];
	finger_angles[1] = finger[1];
	finger_angles[2] = finger[2];
	finger_angles[3] = finger[3];

//	hand->initialize();

	while (1) {
		hand->update();
		F1t = tps[0]->getFullData();
		F2t = tps[1]->getFullData();
		F3t = tps[2]->getFullData();
		F4t = tps[3]->getFullData();

//		std::cout << F1t << std::endl;

		for (i = 0; i < 8; i++) {
			for (j = 0; j < 3; j++) {
				F1_tact(i, j) = F1t[3 * i + j];
				F2_tact(i, j) = F2t[3 * i + j];
				F3_tact(i, j) = F3t[3 * i + j];
				F4_tact(i, j) = F4t[3 * i + j];
			}
		}
		hand->trapezoidalMove(finger_angles, Hand::GRASP);

		double F1max, F2max, F3max, Fe1, Fe2, Fe3;//, Fe4; // variable for max force and Force error( desired - actual )
		int i;

		F1max = F1t[0];
		F2max = F2t[0];
		F3max = F3t[0];
//		F4max = F4t[0];

		for (i = 1; i < 24; i++) {
			if (F1t[i] > F1max)
				F1max = F1t[i];
			if (F2t[i] > F2max) // finding the maximum value of force
				F2max = F2t[i];
			if (F3t[i] > F3max)
				F3max = F3t[i];
//			if (F4t[i] > F4max)
//				F4max = F4t[i];
		}
		double F_des = finger_pressure_max[0] ;
		int scale_factor = finger_resolution[0];

		// F_des=   ;    // need to be given appropriate value

		Fe1 = F_des - F1max;
		Fe2 = F_des - F2max;
		Fe3 = F_des - F3max;
//		Fe4 = F_des - F4max;
		hjp_t finger_angles_current;
//		Eigen::Matrix<double, 4, 1> Finger_Angles_Current_tmp;
		finger_angles_current = hand->getInnerLinkPosition(); // Getting current position of fingers

//		Finger_Angles_Current_tmp[0] = finger_angles_current[0];
//		Finger_Angles_Current_tmp[1] = finger_angles_current[1];
//		Finger_Angles_Current_tmp[2] = finger_angles_current[2]; // setting current position of fingers
//		Finger_Angles_Current_tmp[3] = finger_angles_current[3];

		finger_angles[0] = finger_angles_current[0] + (Fe1 / scale_factor);
		finger_angles[1] = finger_angles_current[1] + (Fe2 / scale_factor);
		finger_angles[2] = finger_angles_current[2] + (Fe3 / scale_factor); // setting current position of fingers
		finger_angles[3] = finger_angles_current[3];

//		finger_angles[0]=+finger_resolution;
	}
	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}
