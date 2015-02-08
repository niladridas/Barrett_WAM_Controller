/*
 * nilu_slidingmode_J2.cpp
 *
 *  Created on: 30-Jan-2015
 *      Author: nilxwam
 */
#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>
//#include <boost/thread.hpp>
//#include <barrett/thread/null_mutex.h>

using namespace barrett;
using detail::waitForEnter;

#include <Dynamics.hpp>
#include <Sliding_mode_4dof.hpp>
#include <reference_signal.hpp>
//#include <dummy_system.hpp>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	typedef boost::tuple<double, jp_type, jv_type, jp_type, jv_type, jt_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, jp_type, jv_type, jt_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	const double TRANSITION_DURATION = 0.5;
	double amplitude1, omega1;
	Eigen::Matrix4d tmp_lamda;
	tmp_lamda << 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20;
	const Eigen::Matrix4d lamda = tmp_lamda;
	const double coeff = 0.1;
	const double delta = 0.01;
	jp_type startpos(0.0);
	startpos[3] = +3.14;
	amplitude1 = 1;
	omega1 = 0.6;
	const double JT_AMPLITUDE = amplitude1;
	const double OMEGA = omega1;
	bool status = true;
	J_ref<DOF> joint_ref(JT_AMPLITUDE, OMEGA);
	Slidingmode_Controller<DOF> slide(status, lamda, coeff, delta);
	Dynamics<DOF> nilu_dynamics;
	//	Dummy<DOF> nilu_dummy(pm.getExecutionManager());
	//	{
	//	BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
	wam.gravityCompensate();
	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	wam.moveTo(startpos);
//
	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();
//	systems::Ramp time(pm.getExecutionManager(), 1.0);
//	systems::Ramp time(pm.getExecutionManager(), 1.0);
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
//	printf("Press 1");

//	printf("Press 2");
//	sleep(6000);
	systems::connect(tg.output, logger.input);
	systems::connect(time.output, joint_ref.timef);
	systems::connect(wam.jpOutput, slide.feedbackjpInput);
	systems::connect(wam.jvOutput, slide.feedbackjvInput);
	systems::connect(wam.jpOutput, nilu_dynamics.jpInputDynamics);
	systems::connect(wam.jvOutput, nilu_dynamics.jvInputDynamics);
	systems::connect(nilu_dynamics.MassMAtrixOutput, slide.M);
	systems::connect(nilu_dynamics.CVectorOutput, slide.C);
	systems::connect(joint_ref.referencejpTrack, slide.referencejpInput);
	systems::connect(joint_ref.referencejvTrack, slide.referencejvInput);
	systems::connect(joint_ref.referencejaTrack, slide.referencejaInput);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput,
			tg.template getInput<4>());
	systems::connect(wam.supervisoryController.output, tg.template getInput<5>());

//	systems::connect(slide.controlOutput, nilu_dummy.input);
//	systems::connect(slide.controlOutput, wam.supervisoryController.input);
//	systems::connect(time.output, tg.template getInput<0>());
//	systems::connect(slide.controlOutput, tg.template getInput<1>());
//	wam.trackReferenceSignal(nilu_dummy.output);
//	printf("Press 3");
	wam.supervisoryController.connectInputTo(slide.controlOutput);
//	printf("Press 4");
	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
//	double abc;
//	std::cin >> abc;
	waitForEnter();
//	printf("It all chikapikaaaaaa");
//	double abc;
//	std::cin >> abc;
//	//
//	status = false;
//	sleep(6000);
//	printf("It all uuuuuuuuuuuaaaaaa");

//	wam.supervisoryController.disconnectInput();
//	printf("It all huuuuuuuuu");
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
//	usleep(5000);
//	printf("It all hiiiiiiii");
//	wam.supervisoryController.disconnectInput();
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	printf("It all ended");

	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);
	return 0;
}

