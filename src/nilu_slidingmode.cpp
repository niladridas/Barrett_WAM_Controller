/*
 * nilu_slidingmode_J2.cpp
 *
 *  Created on: 30-Jan-2015
 *      Author: nilxwam
 */

//#include <iostream>
//#include <string>
//#include <barrett/units.h>
//#include <barrett/systems.h>
//#include <barrett/products/product_manager.h>
//#include <barrett/detail/stl_utils.h>
//#include <barrett/log.h>
//#include <barrett/standard_main_function.h>
//#include <cmath>
//#include <barrett/systems/abstract/system.h>
//
//#include <barrett/math/traits.h>
//#include <list>
//#include <barrett/detail/ca_macro.h>


#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>

using namespace barrett;
using detail::waitForEnter;

#include <Dynamics.hpp>
#include <Sliding_mode_4dof.hpp>
#include <reference_signal.hpp>


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	typedef boost::tuple<double> tuple_type;
	typedef systems::TupleGrouper<double> tg_type;
	tg_type tg;

	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	wam.gravityCompensate();

	const double TRANSITION_DURATION = 0.5; // seconds

	double amplitude1, omega1;
	Eigen::Matrix4d tmp_lamda;
	tmp_lamda << 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20;
	const Eigen::Matrix4d lamda = tmp_lamda;
	const double coeff = 0.1;
	const double delta = 0.01;

	jp_type startpos(0.0);
	startpos[3] = +3.14;

//	std::cout
//			<< "Enter the amplitude of the sinusoid for the joint position of J2: "
//			<< std::endl;
//	std::cin >> amplitude1;
//
//	std::cout
//			<< "Enter the omega of the sinusoid for the joint position of J2: "
//			<< std::endl;
//	std::cin >> omega1;
//
//	std::cout
//			<< "Enter the offset of the sinusoid for the joint position of J2 in degrees: "
//			<< std::endl;
//	std::cin >> offset1;
	amplitude1 = 1.40;
	omega1 = 1;

	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();
	wam.moveTo(startpos);
//
//	std::string Key1;
	printf("Press [Enter] to turn on torque control to joint 2.");
//	std::cin >> Key1;
	waitForEnter();

	systems::Ramp time(pm.getExecutionManager(), 1.0);

	const double JT_AMPLITUDE = amplitude1;
	const double OMEGA = omega1;

	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
//
	J_ref<DOF> joint_ref(JT_AMPLITUDE, OMEGA); /*Offset is in degrees*/
	Slidingmode_Controller<DOF> slide(lamda, coeff, delta);
	Dynamics<DOF> nilu_dynamics;


	systems::connect(time.output, joint_ref.timef);
	systems::connect(wam.jpOutput, nilu_dynamics.jpInputDynamics);
	systems::connect(wam.jvOutput, nilu_dynamics.jvInputDynamics);
	systems::connect(joint_ref.referencejpTrack, slide.referencejpInput);
	systems::connect(joint_ref.referencejvTrack, slide.referencejvInput);
	systems::connect(joint_ref.referencejaTrack, slide.referencejaInput);
	systems::connect(wam.jpOutput, slide.feedbackjpInput);
	systems::connect(wam.jvOutput, slide.feedbackjvInput);
	systems::connect(nilu_dynamics.MassMAtrixOutput, slide.M);
	systems::connect(nilu_dynamics.CVectorOutput, slide.C);

	systems::connect(tg.output, logger.input);
	systems::connect(time.output, tg.template getInput<0>());
//	systems::connect(slide.controlOutput, tg.template getInput<1>());

//	wam.trackReferenceSignal(slide.controlOutput);
	time.smoothStart(TRANSITION_DURATION);

//	std::string Key2;
	printf("Press [Enter] to stop.");
	waitForEnter();
//	std::cin >> Key2;
//	logger.closeLog();

	time.smoothStop(TRANSITION_DURATION);
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);

	return 0;
}

