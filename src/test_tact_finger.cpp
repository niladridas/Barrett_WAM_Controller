/*
 * test_tact_finger.cpp
 *
 *  Created on: 20-Mar-2015
 *      Author: nilxwam
 */

#include <curses.h>
#include<stdlib.h>
#include <barrett/os.h>  // For btsleep()
#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>
#include <Hand_forcetorque_sense.hpp>
#include <Hand_tactile_sense.hpp>
#include <main_processor.hpp>

using namespace barrett;
void waitForEnter() {
	std::string line;
	std::getline(std::cin, line);
}

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam.gravityCompensate();

	// Is an FTS attached?
	ForceTorqueSensor* fts = NULL;
	if (pm.foundForceTorqueSensor()) {
		fts = pm.getForceTorqueSensor();
		fts->tare();
	}

	// Is a Hand attached?
	Hand* hand = NULL;
	std::vector<TactilePuck*> tps;
	if (pm.foundHand()) {
		hand = pm.getHand();

		printf(
				">>> Press [Enter] to initialize Hand. (Make sure it has room!)");
		waitForEnter();
		hand->initialize();
	}

	tps = hand->getTactilePucks();
	// TODO write some error statement
	bool Release_Mode = 0;
	double delta_step = pm.getExecutionManager()->getPeriod();
	double spread_angle = (2.0/3.0)*M_PI;
	std::string  threshold_impulse_str;
	std::cout << "Enter the inpulse threshold limit: ";
	std::cin >> threshold_impulse_str;
	std::cout << "\n" << std::endl;
	double threshold_impulse = atof(threshold_impulse_str.c_str());

	Hand_forcetorque_sense<DOF> hand_ft(hand, fts);
	Hand_tactile_sense hand_tact(hand, tps);
	main_processor<DOF> brain(hand, delta_step, spread_angle, threshold_impulse,
			Release_Mode);

}

