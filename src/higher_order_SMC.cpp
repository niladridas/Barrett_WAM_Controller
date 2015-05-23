/*
 * higher_order_SMC.cpp
 *
 *  Created on: 22-May-2015
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
#include <SMC_higher_order.hpp>
#include <reference_signal.hpp>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);


	Eigen::MatrixXd Coeff;
	Eigen::VectorXd Delta;
	Eigen::VectorXd Amp;
	Eigen::VectorXd Freq;
	Eigen::VectorXd StartPos;
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::VectorXd P;
	Eigen::VectorXd Q;

	Sam::initEigenMat<double>(A, Sam::readFile<double>("A.txt"));
	Sam::initEigenMat<double>(B, Sam::readFile<double>("B.txt"));
	Sam::initEigenVec<double>(P, Sam::readFile<double>("P.txt"));
	Sam::initEigenVec<double>(Q, Sam::readFile<double>("Q.txt"));

	Sam::initEigenMat<double>(Coeff, Sam::readFile<double>("coeff.txt"));
	Sam::initEigenVec<double>(Delta, Sam::readFile<double>("delta.txt"));
	Sam::initEigenVec<double>(Amp, Sam::readFile<double>("amp.txt"));
	Sam::initEigenVec<double>(Freq, Sam::readFile<double>("freq.txt"));
	Sam::initEigenVec<double>(StartPos, Sam::readFile<double>("start.txt"));

	typedef boost::tuple<double, jp_type, jv_type, jp_type, jv_type, jt_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, jp_type, jv_type,
			jt_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	const double TRANSITION_DURATION = 0.5;
	double amplitude1, omega1;
	const Eigen::Matrix4d coeff = Coeff;
	const Eigen::Vector4d delta = Delta;
	jp_type startpos(0.0);


	startpos[0] = StartPos[0];
	startpos[1] = StartPos[1];
	startpos[2] = StartPos[2];
	startpos[3] = StartPos[3];

	const Eigen::Vector4d JP_AMPLITUDE = Amp;
	const Eigen::Vector4d OMEGA = Freq;
	bool status = true;
	const Eigen::Matrix4d A1 = A;
	const Eigen::Matrix4d B1 = B;
	const float P1 = P[0];
	const float Q1 = Q[0];

	J_ref<DOF> joint_ref(JP_AMPLITUDE, OMEGA, startpos);
	SMC_higher_order<DOF> slide(status, coeff, delta , A1, B1,P1, Q1 );
	Dynamics<DOF> nilu_dynamics;

	wam.gravityCompensate();
	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();

	wam.moveTo(startpos);
	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();
	printf("Error 1 \n");

	systems::Ramp time(pm.getExecutionManager(), 1.0);
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
	printf("Error 2 \n");

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

	wam.trackReferenceSignal(slide.controlOutput);
	printf("Error 3 \n");

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput, tg.template getInput<4>());
	systems::connect(slide.controlOutput, tg.template getInput<5>());
	printf("Error 4 \n");

	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();
	printf("Error 5 \n");

	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Error 6 \n");

	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);

	return 0;
}

