/*
 * interactive_torque.cpp
 *
 *  Created on: 24-Jan-2015
 *      Author: nilxwam
 */




#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>

using namespace barrett;
using detail::waitForEnter;



template<size_t DOF>
class J3control: public systems::SingleIO<double,
		typename units::JointTorques<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	J3control(jt_type startTorque, double amplitude, double omega,
			const std::string& sysName = "JtSinusoid") :
			systems::SingleIO<double, jt_type>(sysName), jt(startTorque), j2_0(
					jt[1]), amp(amplitude), omega(omega) {
	}
	virtual ~J3control() {
		this->mandatoryCleanUp();
	}

protected:
	jt_type jt;
	double j2_0;
	double amp, omega;
	double theta;

	virtual void operate() {
		theta = omega * this->input.getValue();
		jt[1] = amp* std::sin(theta) + j2_0;
//		jt[3] = 3.0 * std::sin(theta) + j4_0;
		//jt[3] = 1.5;

		this->outputValue->setData(&jt);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(J3control);
};





template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

//
	typedef boost::tuple<jt_type, jp_type, jv_type, jt_type, jt_type> tuple_type;
	typedef systems::TupleGrouper<jt_type, jp_type, jv_type, jt_type, jt_type> tg_type;
	tg_type tg;

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
//
	double Frequency;
	std::cout << "Enter Frequency ";
	std::cin >> Frequency;
	std::cout << "\n Enter Amplitude ";
	double Amplitude;
	std::cin >> Amplitude;

//	const double JT_AMPLITUDE = 3; // N*m
//	const double FREQUENCY =1; // rad/s
//
	if (Amplitude*Frequency > 5 && Frequency > 3)
	{
		Amplitude = 4;
		Frequency = 1;
	}


	const double JT_AMPLITUDE = Amplitude;
	const double FREQUENCY = Frequency;


	wam.gravityCompensate();

	const double TRANSITION_DURATION = 0.5; // seconds



	jt_type startTorque(0.0);
	jp_type startpos(0.0);
	startpos[3] = +3.14;
	systems::Ramp time(pm.getExecutionManager(), 1.0);

	printf("Press [Enter] to turn on torque control to go to zero position");
	wam.moveTo(startpos);


	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();

	//
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
	//

	J3control<DOF> jtc(startTorque, JT_AMPLITUDE, FREQUENCY);
	systems::connect(tg.output, logger.input);
	systems::connect(time.output, jtc.input);
	systems::connect(jtc.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(wam.jvOutput, tg.template getInput<2>());
	systems::connect(wam.gravity.output,tg.template getInput<3>());
	systems::connect(wam.jtSum.output, tg.template getInput<4>());



	wam.trackReferenceSignal(jtc.output);
	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
		lr.exportCSV(argv[1]);
		printf("Output written to %s.\n", argv[1]);
		std::remove(tmpFile);
	return 0;
}
