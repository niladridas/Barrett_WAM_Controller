/*
 * test_tact_finger.cpp
 *
 *  Created on: 20-Mar-2015
 *      Author: nilxwam
 */

#include <curses.h>

#include <barrett/os.h>  // For btsleep()
#include <barrett/math.h>  // For barrett::math::saturate()
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>

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

	if (hand->hasTactSensors()) {
		tps = hand->getTactilePucks();
	} else {
		printw(" n/a");
		// TODO write some error statement
	}

}

