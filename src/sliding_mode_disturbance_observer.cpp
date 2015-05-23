/*
 * sliding_mode_disturbance_observer.cpp
 *
 *  Created on: 07-May-2015
 *      Author: nilxwam
 */
//#include <eigen3/Eigen/Dense>
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
#include <reference_signal.hpp>
#include <dummy_system.hpp>
#include <GMM.hpp>
#include <differentiator.hpp>
#include <second_differentiator.hpp>
#include <torque_observer.hpp>
#include <dummy_system.hpp>
#include <disturbance_observer.hpp>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	Eigen::MatrixXd Lamda;

	Eigen::MatrixXd Coeff;
	Eigen::VectorXd Delta;
	Eigen::VectorXd Amp;
	Eigen::VectorXd Freq;
	Eigen::VectorXd StartPos;
	Eigen::VectorXd dist_K_tmp;
	Eigen::VectorXd state_intg;
	Eigen::VectorXd A_tmp;


	Sam::initEigenMat<double>(Lamda, Sam::readFile<double>("lamda.txt"));
	Sam::initEigenMat<double>(Coeff, Sam::readFile<double>("coeff.txt"));
	Sam::initEigenVec<double>(Delta, Sam::readFile<double>("delta.txt"));
	Sam::initEigenVec<double>(Amp, Sam::readFile<double>("amp.txt"));
	Sam::initEigenVec<double>(Freq, Sam::readFile<double>("freq.txt"));
	Sam::initEigenVec<double>(StartPos, Sam::readFile<double>("start.txt"));
	Sam::initEigenVec<double>(dist_K_tmp, Sam::readFile<double>("dist_K.txt"));
	Sam::initEigenVec<double>(state_intg, Sam::readFile<double>("integrator_state.txt"));
	Sam::initEigenVec<double>(A_tmp, Sam::readFile<double>("A.txt"));



	std::cout << "Lamda is" << Lamda << "\n" << "Coeff is" << Coeff << "\n"
			<< "Delta is" << Delta << "\n" << std::endl;
	typedef boost::tuple<double, jp_type, jv_type, jp_type, jv_type, jt_type,
			jt_type, double, jp_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, jp_type, jv_type,
			jt_type, jt_type, double, jp_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	const double TRANSITION_DURATION = 0.5;
//	double amplitude1, omega1;
	const Eigen::Matrix4d lamda = Lamda;

	const Eigen::Matrix4d coeff = Coeff;
	const Eigen::Vector4d delta = Delta;
	jp_type startpos(0.0);

	startpos[0] = StartPos[0];
	startpos[1] = StartPos[1];
	startpos[2] = StartPos[2];
	startpos[3] = StartPos[3];

	const Eigen::Vector4d JP_AMPLITUDE = Amp;
	const Eigen::Vector4d OMEGA = Freq;
	Eigen::Vector4d tmp_velocity;
	tmp_velocity[0] = Amp[0]*Freq[0];
	tmp_velocity[1] = Amp[1]*Freq[1];
	tmp_velocity[2] = Amp[2]*Freq[2];
	tmp_velocity[3] = Amp[3]*Freq[3];
	bool status = true;
	const double dist_K =  dist_K_tmp[0];
	const int state = state_intg[0];
	const Eigen::Vector4d initial_velocity = tmp_velocity;

	const float A = A_tmp[0];

	J_ref<DOF> joint_ref(JP_AMPLITUDE, OMEGA, startpos);
	Slidingmode_Controller<DOF> slide(status, lamda, coeff, delta);
	Dynamics<DOF> nilu_dynamics;
	disturbance_observer<DOF> nilu_disturbance(dist_K,state, initial_velocity,A);
	Dummy<DOF> nilu_dummy;

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

	systems::connect(time.output, nilu_disturbance.time);
	systems::connect(nilu_dynamics.MassMAtrixOutput, nilu_disturbance.M);
	systems::connect(nilu_dynamics.CVectorOutput, nilu_disturbance.C);
	systems::connect(wam.jpOutput, nilu_disturbance.inputactual_pos);
	systems::connect(wam.jvOutput, nilu_disturbance.inputactual_vel);
	systems::connect(slide.controlOutput, nilu_disturbance.SMC);

	systems::connect(slide.controlOutput, nilu_dummy.ControllerInput);
	systems::connect(nilu_disturbance.disturbance_torque_etimate,
			nilu_dummy.CompensatorInput);

	wam.trackReferenceSignal(nilu_dummy.Total_torque);
	printf("Error 3 \n");

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput, tg.template getInput<4>());
	systems::connect(nilu_disturbance.disturbance_torque_etimate,
			tg.template getInput<5>());
	systems::connect(slide.controlOutput, tg.template getInput<6>());
	systems::connect(nilu_disturbance.Test, tg.template getInput<7>());
	systems::connect(nilu_disturbance.InverseTest, tg.template getInput<8>());



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

