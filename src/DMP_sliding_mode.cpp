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

#include <dummy_system.hpp>
#include <GMM.hpp>
#include <differentiator.hpp>
#include <second_differentiator.hpp>
#include <torque_observer.hpp>
#include <dummy_system.hpp>
#include <DMP_first.h>
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	printf("Error 1 \n");

	Eigen::MatrixXd Lamda;

	Eigen::MatrixXd Coeff;
	Eigen::VectorXd Delta;
	Eigen::VectorXd Amp;
	Eigen::VectorXd Freq;
	Eigen::VectorXd StartPos;

	Sam::initEigenMat<double>(Lamda, Sam::readFile<double>("lamda.txt"));
	Sam::initEigenMat<double>(Coeff, Sam::readFile<double>("coeff.txt"));
	Sam::initEigenVec<double>(Delta, Sam::readFile<double>("delta.txt"));
	Sam::initEigenVec<double>(Amp, Sam::readFile<double>("amp.txt"));
	Sam::initEigenVec<double>(Freq, Sam::readFile<double>("freq.txt"));
	Sam::initEigenVec<double>(StartPos, Sam::readFile<double>("start.txt"));

	typedef boost::tuple<double, jp_type, jv_type, ja_type, jp_type, jv_type,
			jt_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, ja_type, jp_type,
			jv_type, jt_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	const double TRANSITION_DURATION = 0.5;
	double amplitude1, omega1;
	const Eigen::Matrix4d lamda = Lamda;
	const Eigen::Matrix4d coeff = Coeff;
	const Eigen::Vector4d delta = Delta;
	jp_type startpos(0.0);
	startpos[0] = StartPos[0];
	startpos[1] = StartPos[1];
	startpos[2] = StartPos[2];
	startpos[3] = StartPos[3];

	bool status = true;


	float a[4] = { 25.0, 25.0, 25.0, 25.0 }; // PD values
	float b[4] = { 25.0 / 4.0, 25.0 / 4.0, 25.0 / 4.0, 25.0 / 4.0 }; // PD values
	float y0[4] = { 0.339813, -1.00531, -2.05592, 1.21244 }; // Initial state [x0,y0,z0]
	float goal[4] = { 0, 0.00260865, 0.000410464, 0.000326016 };
	DMP_first<DOF> DMP_first(4, 500, a, b, 9.134, 0.05, y0, goal);
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

	wam.trackReferenceSignal(slide.controlOutput);

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
	lr.close();
	return 0;
}

