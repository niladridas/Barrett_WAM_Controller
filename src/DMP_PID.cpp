/*
 * DMP_PID.cpp
 *
 *  Created on: 04-Jun-2015
 *      Author: nilxwam
 */

//--------------------------------------------------
// HEADER FILES


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
//#include <samlibs.h>

//#include <Dynamics.hpp>
//#include <Sliding_mode_4dof.hpp>

//#include <dummy_system.hpp>
//#include <GMM.hpp>
//#include <differentiator.hpp>
//#include <second_differentiator.hpp>
//#include <torque_observer.hpp>
//#include <dummy_system.hpp>
//#include <DMP_first.h>
//
//
////--------------------------------------------------
//#include <unistd.h>
//#include <iostream>
//#include <string>
//#include <barrett/units.h>
//#include <barrett/systems.h>
//#include <barrett/products/product_manager.h>
//#include <barrett/detail/stl_utils.h>
//#include <barrett/log.h>
//#include <barrett/standard_main_function.h>
//#include <samlibs.h>
#include <DMP.h>
//--------------------------------------------------

//--------------------------------------------------
// HEADER FILES
//--------------------------------------------------
//using namespace barrett;
//using detail::waitForEnter;
//--------------------------------------------------

//--------------------------------------------------
// MAIN
//--------------------------------------------------
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
//--------------------------------------------------

	typedef boost::tuple<double, jp_type, jv_type, ja_type, jp_type, jv_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, ja_type, jp_type,
			jv_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	const double TRANSITION_DURATION = 0.5;
//--------------------------------------------------
// DMP INITIALIZATION
//--------------------------------------------------

	float a[1] = { 25.0 }; // PD values
	float b[1] = { 25.0 / 4.0 }; // PD values
	float y0[1] = { -1.41145 }; // Initial state [x0,y0,z0]
	float goal[1] = { 0.960598 };
	DMP<DOF> DMP(1, 500, a, b, 3.532, 0.05, y0, goal);
//	DMP_first<DOF> DMP_first(DMPs, BFs, Ay, By, Final_Time, Steady_State_Tolerance, Initial_Pose, Final_Pose);
//--------------------------------------------------

//--------------------------------------------------
// ROBOT HOMING
//--------------------------------------------------
	jp_type startpos(0.0);
	startpos[0] = 0;
	startpos[1] = -1.41145;
	startpos[2] = 0;
	startpos[3] = 3.14;

	wam.gravityCompensate();
	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();

	wam.moveTo(startpos);
	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();
//--------------------------------------------------

//--------------------------------------------------
// TIME MANAGER
//--------------------------------------------------
	systems::Ramp time(pm.getExecutionManager(), 1.0);

	const size_t PERIOD_MULTIPLIER = 1;

	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	systems::connect(tg.output, logger.input);
//--------------------------------------------------

//--------------------------------------------------
// MAIN CONNECTION
//--------------------------------------------------
//	systems::connect(tg.output, logger.input);
	systems::connect(wam.jpOutput, DMP.CurrentStatePos);
	systems::connect(wam.jvOutput, DMP.CurrentStateVel);
	wam.trackReferenceSignal(DMP.ref_jp);
//--------------------------------------------------

//--------------------------------------------------
// LOGGER CODE
//--------------------------------------------------
	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(DMP.ref_jp, tg.template getInput<1>());
	systems::connect(DMP.ref_jv, tg.template getInput<2>());
	systems::connect(DMP.ref_ja, tg.template getInput<3>());
	systems::connect(wam.jpOutput, tg.template getInput<4>());
	systems::connect(wam.jvOutput, tg.template getInput<5>());
//--------------------------------------------------

//--------------------------------------------------
// CLOSING CODE
//--------------------------------------------------
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
	lr.close();
//--------------------------------------------------

	return 0;
}

