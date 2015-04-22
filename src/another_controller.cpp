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
#include<fstream>
using namespace std;
using namespace barrett;
using detail::waitForEnter;
#include <samlibs.h>

#include <Dynamics.hpp>
#include <backstepping.hpp>
#include <reference_signal.hpp>
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
	typedef boost::tuple<double, jp_type, jv_type, jp_type, jv_type, jt_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, jp_type, jv_type,
			jt_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}


	Eigen::VectorXd Amp;
	Eigen::VectorXd Freq;
	Eigen::VectorXd StartPos;


	Sam::initEigenVec<double>(Amp, Sam::readFile<double>("amp.txt"));
	Sam::initEigenVec<double>(Freq, Sam::readFile<double>("freq.txt"));
	Sam::initEigenVec<double>(StartPos, Sam::readFile<double>("start.txt"));

	Eigen::Matrix4d K1;
	Eigen::Matrix4d K2;
	ifstream myReadFile;
	myReadFile.open("data/K1.txt");
	double W1[16];
	for (int i = 0; i < 16; i++) {
		myReadFile >> W1[i];
	}
	myReadFile.close();
	K1 << W1[0], W1[1], W1[2], W1[3], W1[4], W1[5], W1[6], W1[7], W1[8], W1[9], W1[10], W1[11], W1[12], W1[13], W1[14], W1[15];

	ifstream myReadFile1;
	myReadFile1.open("data/K2.txt");
	double W2[16];
	for (int i = 0; i < 16; i++) {
		myReadFile1 >> W2[i];
	}
	myReadFile1.close();
	K2 << W2[0], W2[1], W2[2], W2[3], W2[4], W2[5], W2[6], W2[7], W2[8], W2[9], W2[10], W2[11], W2[12], W2[13], W2[14], W2[15];

	std::cout << K1 << "\n" << K2 << "\n"<< std::endl;

	const double TRANSITION_DURATION = 0.5;
	double amplitude1, omega1;
	Eigen::Matrix4d tmp_lamda;
	tmp_lamda << 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20, 0, 0, 0, 0, 20;
	const Eigen::Matrix4d lamda = tmp_lamda;

	jp_type startpos(0.0);
	startpos[0] = StartPos[0];
	startpos[1] = StartPos[1];
	startpos[2] = StartPos[2];
	startpos[3] = StartPos[3];
//	std::cout << "Enter amplitude of ref sinusoid: " << std::endl;
//	std::cin >> amplitude1;
//	std::cout << "Enter omega of ref sinusoid: " << std::endl;
//	std::cin >> omega1;
//	startpos[3] = +3.14;

	const 	Eigen::VectorXd JP_AMPLITUDE = Amp;
	const	Eigen::VectorXd OMEGA = Freq;
	bool status = true;

	Eigen::Matrix4d lambda;
	lambda << 200, 0, 0, 0, 0, 200, 0, 0, 0, 0, 200, 0, 0, 0, 0, 200;

	J_ref<DOF> joint_ref(JP_AMPLITUDE, OMEGA, startpos);
	Backstepping_Controller<DOF> slide(K1, K2);
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
//	systems::connect(time.output, nilu_diff.time);
//	systems::connect(time.output, nilu_diff2.time);
//	systems::connect(wam.jvOutput, nilu_diff.inputSignal);
//	systems::connect(wam.jpOutput,nilu_diff2.inputSignal);

//	systems::connect(time.output, nilu_TO.time);
//	systems::connect(nilu_diff.outputSignal, nilu_TO.inputacc);
//	systems::connect(nilu_dynamics.MassMAtrixOutput, nilu_TO.M);
//	systems::connect(nilu_dynamics.CVectorOutput, nilu_TO.C);
//	systems::connect(slide.controlOutput, nilu_TO.SMC);
//	systems::connect(nilu_TO.outputTorque,nilu_dummy.CompensatorInput);
//	systems::connect(slide.controlOutput,nilu_dummy.ControllerInput);

//	wam.trackReferenceSignal(nilu_dummy.Total_torque);

	wam.trackReferenceSignal(slide.controlOutput);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput, tg.template getInput<4>());
	systems::connect(slide.controlOutput, tg.template getInput<5>());
//	systems::connect(nilu_diff.outputSignal, tg.template getInput<6>());
//	systems::connect(nilu_TO.outputTorque, tg.template getInput<7>());
//	systems::connect(nilu_diff2.outputSignal, tg.template getInput<7>());

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

