/*
 * ex08_teach_and_play.cpp
 *
 *  Created on: Sep 29, 2009
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#include <barrett/standard_main_function.h>

using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	typedef boost::tuple<double, jp_type, jv_type> jp_sample_type;

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	const double T_s = pm.getExecutionManager()->getPeriod();

	wam.gravityCompensate();

	systems::Ramp time(pm.getExecutionManager(), 1.0);

	systems::TupleGrouper<double, jp_type, jv_type> jpLogTg;

	// Record at 1/10th of the loop rate
	systems::PeriodicDataLogger<jp_sample_type> jpLogger(
			pm.getExecutionManager(),
			new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, T_s), 1);

	time.setOutput(0.0);
	std::string count_str;
	count_str = argv[2];
	int count_num;
	count_num = atoi(count_str.c_str());

	for (size_t i = 0; i < count_num; i++) {
		printf("Press [Enter] to start teaching.\n");
		waitForEnter();
		{

			BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());

			connect(time.output, jpLogTg.template getInput<0>());
			connect(wam.jpOutput, jpLogTg.template getInput<1>());
			connect(wam.jvOutput, jpLogTg.template getInput<2>());
			time.start();
			connect(jpLogTg.output, jpLogger.input);

		}

		printf("Press [Enter] to stop teaching.\n");
		waitForEnter();
		disconnect(jpLogTg.template getInput<0>());
		disconnect(jpLogTg.template getInput<1>());
		disconnect(jpLogTg.template getInput<2>());
		disconnect(jpLogger.input);
		time.stop();
		time.setOutput(0.0);

	}
	jpLogger.closeLog();

	// Build spline between recorded points
	log::Reader<jp_sample_type> lr(tmpFile);
	lr.exportCSV(argv[1]);

	std::remove(tmpFile);
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);

	return 0;
}
