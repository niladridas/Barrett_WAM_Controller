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
#include <samlibs.h>

#include <Dynamics.hpp>
#include <Sliding_mode_4dof.hpp>
#include <DMP_first.h>
#include <dummy_system.hpp>
#include <GMM.hpp>
#include <differentiator.hpp>
#include <second_differentiator.hpp>
#include <torque_observer.hpp>
#include <dummy_system.hpp>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	printf("Error 1 \n");

	typedef boost::tuple<double, jp_type, jv_type,ja_type, jp_type, jv_type, jt_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type,ja_type, jp_type, jv_type, jt_type> tg_type;
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

	double COEFF;
	std::cout << "Enter Coeff: " << std::endl;
	std::cin >> COEFF;
	const double coeff = COEFF;
	double DELTA;
	std::cout << "Enter delta: " << std::endl;
	std::cin >> DELTA;
	const double delta = DELTA;
	jp_type startpos(0.0);
	startpos[1] = -1.41145;
	startpos[3] = M_PI;

	bool status = true;

	Eigen::Matrix4d lambda;
	lambda << 200, 0, 0, 0, 0, 200, 0, 0, 0, 0, 200, 0, 0, 0, 0, 200;

	float a[1] = { 30.0 }; // PD values
	float b[1] = { 30.0 / 4.0 }; // PD values
	float y0[1] = {-1.41145}; // Initial state [x0,y0,z0]
	float goal[1] = {0.960598};
	DMP_first<DOF> DMP_first(1, 500, a, b, 4.16, 0.05, y0,goal);
	Slidingmode_Controller<DOF> slide(status, lamda, coeff, delta);
	Dynamics<DOF> nilu_dynamics;

	wam.gravityCompensate();
	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();

	wam.moveTo(startpos);
	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	systems::connect(tg.output, logger.input);
	systems::connect(wam.jpOutput, slide.feedbackjpInput);
	systems::connect(wam.jvOutput, slide.feedbackjvInput);
	systems::connect(wam.jpOutput, nilu_dynamics.jpInputDynamics);
	systems::connect(wam.jvOutput, nilu_dynamics.jvInputDynamics);
	systems::connect(nilu_dynamics.MassMAtrixOutput, slide.M);
	systems::connect(nilu_dynamics.CVectorOutput, slide.C);
	systems::connect(DMP_first.ref_jp, slide.referencejpInput);
	systems::connect(DMP_first.ref_jv, slide.referencejvInput);
	systems::connect(DMP_first.ref_ja, slide.referencejaInput);


//	wam.trackReferenceSignal(slide.controlOutput);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(DMP_first.ref_jp, tg.template getInput<1>());
	systems::connect(DMP_first.ref_jv, tg.template getInput<2>());
	systems::connect(DMP_first.ref_ja, tg.template getInput<3>());
	systems::connect(wam.jpOutput, tg.template getInput<4>());
	systems::connect(wam.jvOutput, tg.template getInput<5>());
	systems::connect(slide.controlOutput, tg.template getInput<6>());

	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);
	return 0;
}

