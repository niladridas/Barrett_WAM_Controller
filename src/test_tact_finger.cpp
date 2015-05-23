/*
 * test_tact_finger.cpp
 *
 *  Created on: 20-Mar-2015
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
#include <Hand_tactile_sense.hpp>
#include <main_processor.hpp>
#include <Hand_full_move.hpp>
#include <barrett/log.h>

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

	typedef Hand::jp_type hjp_t;

	typedef boost::tuple<double, hjp_t> tuple_type;
	typedef systems::TupleGrouper<double, hjp_t> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	const double TRANSITION_DURATION = 0.5;

	wam.gravityCompensate();

//	printf("Press [Enter] to  go to given position");
//	waitForEnter();
//	jp_type startpos(0.0); // TODO : change here
//	wam.moveTo(startpos);

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
		hand->trapezoidalMove(Hand::jp_type((1.0 / 3.0) * M_PI), Hand::SPREAD);
		hand->trapezoidalMove(Hand::jp_type((1.0 / 3.0) * M_PI), Hand::GRASP);
		hand->trapezoidalMove(Hand::jp_type((0.0) * M_PI), Hand::GRASP);

	}
	printf("Error 1 \n");
	tps = hand->getTactilePucks();
	// TODO write some error statement
	bool Release_Mode = 0;
	double delta_step = 0.002; //pm.getExecutionManager()->getPeriod();
	std::string input_angle_string;
	input_angle_string = argv[1];

	double input_angle = atoi(input_angle_string.c_str());
	double spread_angle = (input_angle / 180.0) * M_PI;
//	std::string threshold_impulse_str;
//	std::cout << "Enter the inpulse threshold limit: ";
//	std::cin >> threshold_impulse_str;
//	std::cout << "\n" << std::endl;
	std::string threshold_impulse_string;
	threshold_impulse_string = argv[2];

	double threshold_impulse = atof(threshold_impulse_string.c_str());

	printf("Press [Enter] to turn on the system");
	waitForEnter();
	printf("Error 2 \n");

	systems::Ramp time(pm.getExecutionManager(), 1.0);
//	const size_t PERIOD_MULTIPLIER = 1;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
	printf("Error 3 \n");

//	Hand_forcetorque_sense<DOF> hand_ft(hand, fts);
	Hand_tactile_sense hand_tact(hand, tps);
	main_processor<DOF> brain(hand, delta_step, spread_angle, threshold_impulse,
			Release_Mode);
	Hand_full_move hand_move(hand);

	systems::connect(tg.output, logger.input);
	systems::connect(time.output, brain.Current_time);
//	systems::connect(hand_ft.Force_hand, brain.Force_hand);
//	systems::connect(hand_ft.Torque_hand, brain.Torque_hand);
//	systems::connect(hand_ft.Acceleration_hand, brain.Acceleration_hand);
	systems::connect(hand_tact.Finger_Tactile_1, brain.Finger_Tactile_1);
	systems::connect(hand_tact.Finger_Tactile_2, brain.Finger_Tactile_2);
	systems::connect(hand_tact.Finger_Tactile_3, brain.Finger_Tactile_3);
	systems::connect(hand_tact.Finger_Tactile_4, brain.Finger_Tactile_4);
	systems::connect(brain.Desired_Finger_Angles, hand_move.Finger_Angles);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(brain.Desired_Finger_Angles, tg.template getInput<1>());
//	systems::connect(hand_ft.Force_hand_cf, tg.template getInput<1>());


	printf("Error 4 \n");

	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();

	printf("Error 5 \n");
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();
	printf("Error 6 \n");
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[3]);
	printf("Error 7 \n");
	printf("Output written to %s.\n", argv[3]);
	std::remove(tmpFile);
	return 0;

}

