/*
 * nilu_optimal_sliding_mode.cpp
 *
 *  Created on: 25-Feb-2015
 *      Author: nilxwam
 */

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
#include<fstream>
//#include <boost/thread.hpp>
//#include <barrett/thread/null_mutex.h>

using namespace barrett;
using detail::waitForEnter;
#include <samlibs.h>

#include <Dynamics_mod.hpp>
#include <Optimal_Sliding_Mode.hpp>
#include <reference_signal.hpp>
#include <dummy_system.hpp>
//#include <GMM.hpp>

using namespace std;

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

//	typedef Eigen::Matrix<double, 7, 1> Vector7d;
	printf("Error 1 \n");

	typedef boost::tuple<double, jp_type, jv_type, jp_type, jv_type, jt_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, jp_type, jv_type,
			jt_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	printf("Error 2 \n");

	const double TRANSITION_DURATION = 0.5;
	double amplitude1, omega1;

	jp_type startpos(0.0);

	std::cout << "Enter amplitude of ref sinusoid: " << std::endl;
	std::cin >> amplitude1;
	std::cout << "Enter omega of ref sinusoid: " << std::endl;
	std::cin >> omega1;
	startpos[3] = +3.14;

	const double JP_AMPLITUDE = amplitude1;
	const double OMEGA = omega1;

	J_ref<DOF> joint_ref(JP_AMPLITUDE, OMEGA, startpos);

	/* The phi and weight values need to be assigned initial values. Then, they are to be updated */
//			Initial conditions of weights and phi
	Eigen::Vector3d W1;
	Eigen::Vector3d W2;
	Eigen::Vector3d W3;
	Eigen::Vector3d W4;


	ifstream myReadFile;
	myReadFile.open("/home/niladri-64/module_heisenberg/data/end_effector_cartesian.txt");
	float output[9];
	for(int i =0 ; i<9; i++)
	{
	myReadFile >> output[i];
	}
	myReadFile.close();


	W1 << 15, 20, 25;
	W2 << 15, 20, 25;
	W3 << 15, 20, 25;
	W4 << 15, 20, 25;
//
	Eigen::Vector4d phi;
	phi << 0, 0.6, 0, 0;
//
	Eigen::Vector4d c;
	c << 20, 20, 20, 20;
//
	Eigen::Vector4d K;
	K << 5, 5, 5, 5;
//
	double b;
	b = 0.1;
//
	double del;
	del = 0.002;
//
	Eigen::Vector3f Gtilde;
	Gtilde << 0, -1, 1;

	Optimal_Sliding_Mode<DOF> optslide(W1, W2, W3, W4, phi, c, K, b, del,
			Gtilde);
	Dynamics_mod<DOF> nilu_dynamics;

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
//	systems::connect(wam.jpOutput, optslide.feedbackjpInput);
//	systems::connect(wam.jvOutput, optslide.feedbackjvInput);

	systems::connect(joint_ref.referencejpTrack,
			nilu_dynamics.referencejpInput);
	systems::connect(joint_ref.referencejvTrack,
			nilu_dynamics.referencejvInput);
	systems::connect(wam.jpOutput, nilu_dynamics.feedbackjpInput);
	systems::connect(wam.jvOutput, nilu_dynamics.feedbackjvInput);

//	systems::connect(nilu_dynamics.MassMAtrixOutput, optslide.M);
//	systems::connect(nilu_dynamics.CVectorOutput, optslide.C);
//	systems::connect(nilu_dynamics.MassMAtrixOutputref, optslide.Md);
//	systems::connect(nilu_dynamics.CVectorOutputref, optslide.Cd);

//	systems::connect(joint_ref.referencejpTrack, optslide.referencejpInput);
//	systems::connect(joint_ref.referencejvTrack, optslide.referencejvInput);
//	systems::connect(joint_ref.referencejaTrack, optslide.referencejaInput);

//	wam.trackReferenceSignal(optslide.controlOutput);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput, tg.template getInput<4>());
//	systems::connect(optslide.controlOutput, tg.template getInput<5>());

	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	printf("It all ended");
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);
	return 0;
}

