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
#include <reference_signal.hpp>
#include <dummy_system.hpp>
#include <GMM.hpp>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	size_t num_inp, num_op, num_priors;
	Eigen::VectorXd priors;/*!< an Eigen Vector value */
	Eigen::MatrixXd mean;/*!< an Eigen Matrix value */
	Eigen::MatrixXd sigma;
	Eigen::VectorXd output;
	Sam::initEigenVec<double>(priors, Sam::readFile<double>("data/priors.txt"));
	Sam::initEigenMat<double>(mean, Sam::readFile<double>("data/mean.txt"));
	Sam::initEigenMat<double>(sigma, Sam::readFile<double>("data/sigma.txt"));

	num_priors = priors.rows();
	std::map<std::string, std::string> allData_inp;

	if (Sam::readConfigFile("data/sysconfig.txt", allData_inp)) {
		if (!Sam::findValueFromMap<size_t>(allData_inp, "num_input", num_inp)) {
			exit(EXIT_FAILURE);
		}
		if (!Sam::findValueFromMap<size_t>(allData_inp, "num_output", num_op)) {
			exit(EXIT_FAILURE);
		}
	} else
		exit(EXIT_FAILURE);
	output.resize(num_op);

	//
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
//	Dummy<DOF> nilu_dummy;
	GMM<DOF> gmm_error(priors,mean,sigma,num_inp,num_op,num_priors); //	{

	wam.gravityCompensate();
	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	wam.moveTo(startpos);
	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();
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
//	systems::connect(slide.controlOutput, nilu_dummy.ControllerInput);
	systems::connect(wam.jpOutput, gmm_error.jpInputActual);
	systems::connect(wam.jvOutput, gmm_error.jvInputActual);
//	systems::connect(gmm_error.error_output, nilu_dummy.CompensatorInput);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput, tg.template getInput<4>());
	systems::connect(gmm_error.error_output,
				tg.template getInput<5>());


//	systems::connect(wam.supervisoryController.output,
//			tg.template getInput<5>());

	wam.supervisoryController.connectInputTo(wam.supervisoryController.output);
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

