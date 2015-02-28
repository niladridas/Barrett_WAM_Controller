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

#include <unistd.h>                  /*  for sleep()  */
#include <curses.h>
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

	ifstream myReadFile1;
	myReadFile1.open("data/W_optimal_sliding.txt");
	double W[12];
	for (int i = 0; i < 12; i++) {
		myReadFile1 >> W[i];
	}
	myReadFile1.close();

	W1 << W[0], W[1], W[2];
	W2 << W[3], W[4], W[5];
	W3 << W[6], W[7], W[8];
	W4 << W[9], W[10], W[11];

	ifstream myReadFile2;
	myReadFile2.open("data/Phi_optimal_sliding.txt");
	double Phi[4];
	for (int i = 0; i < 4; i++) {
		myReadFile2 >> Phi[i];
	}
	myReadFile2.close();
//
	Eigen::Vector4d phi;
	phi << Phi[0], Phi[1], Phi[2], Phi[3];

	ifstream myReadFile3;
	myReadFile3.open("data/c_optimal_sliding.txt");
	double C[4];
	for (int i = 0; i < 4; i++) {
		myReadFile3 >> C[i];
	}
	myReadFile3.close();

//
	Eigen::Vector4d c;
	c << C[0], C[1], C[2], C[3];
//

	ifstream myReadFile4;
	myReadFile4.open("data/K_optimal_sliding.txt");
	double K_optimal[4];
	for (int i = 0; i < 4; i++) {
		myReadFile4 >> K_optimal[i];
	}
	myReadFile4.close();
	Eigen::Vector4d K;
	K << K_optimal[0], K_optimal[1], K_optimal[2], K_optimal[3];
//



	ifstream myReadFile5;
	myReadFile5.open("data/b_del.txt");
	double b_del[2];
	for (int i = 0; i < 2; i++) {
		myReadFile5 >> b_del[i];
	}
	myReadFile5.close();
	double b;
	b = b_del[0];
//
	double del;
	del = b_del[1];
//

	ifstream myReadFile6;
	myReadFile6.open("data/Gtilde.txt");
	double Gtilde_tmp[3];
	for (int i = 0; i < 3; i++) {
		myReadFile6 >> Gtilde_tmp[i];
	}
	myReadFile6.close();
	Eigen::Vector3f Gtilde;
	Gtilde << Gtilde_tmp[0], Gtilde_tmp[1], Gtilde_tmp[2];

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
	systems::connect(wam.jpOutput, optslide.feedbackjpInput);
	systems::connect(wam.jvOutput, optslide.feedbackjvInput);

	systems::connect(joint_ref.referencejpTrack,
			nilu_dynamics.referencejpInput);
	systems::connect(joint_ref.referencejvTrack,
			nilu_dynamics.referencejvInput);
	systems::connect(wam.jpOutput, nilu_dynamics.feedbackjpInput);
	systems::connect(wam.jvOutput, nilu_dynamics.feedbackjvInput);

	systems::connect(nilu_dynamics.MassMAtrixOutput, optslide.M);
	systems::connect(nilu_dynamics.CVectorOutput, optslide.C);
	systems::connect(nilu_dynamics.MassMAtrixOutputref, optslide.Md);
	systems::connect(nilu_dynamics.CVectorOutputref, optslide.Cd);

	systems::connect(joint_ref.referencejpTrack, optslide.referencejpInput);
	systems::connect(joint_ref.referencejvTrack, optslide.referencejvInput);
	systems::connect(joint_ref.referencejaTrack, optslide.referencejaInput);

	wam.trackReferenceSignal(optslide.optslidecontrolOutput);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput, tg.template getInput<4>());
	systems::connect(optslide.optslidecontrolOutput, tg.template getInput<5>());

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

