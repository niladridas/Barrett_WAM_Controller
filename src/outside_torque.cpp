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





//#include <iostream>
//#include <string>
//
//#include <cstdlib>  // For mkstmp()
//#include <cstdio>  // For remove()
//#include <boost/lexical_cast.hpp>
//#include <boost/tuple/tuple.hpp>
//#include <barrett/math.h>
//#include <barrett/log.h>
//#include <barrett/units.h>
//#include <barrett/detail/stl_utils.h>
//#include <barrett/systems.h>
//#include <barrett/products/product_manager.h>
//#include<myUtilWAMwrapper.h>
//#define BARRETT_SMF_VALIDATE_ARGS
//#include <barrett/standard_main_function.h>

using namespace barrett;
using detail::waitForEnter;


//bool validate_args(int argc, char** argv) {
//    if (argc <2) {
//        std::cout << "Usage: " << argv[0] << " <File Write>"<<" <Filter1 cutoff freq.>"<< std::endl;
//        return false;
//    }
//    return true;
//}




template<size_t DOF>
class J2control: public systems::SingleIO<double,
		typename units::JointPositions<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	J2control(jp_type startPosition, double amplitude, double omega,
			const std::string& sysName = "JpSinusoid") :
			systems::SingleIO<double, jp_type>(sysName), jp(startPosition), j2_0(
					jp[3]), amp(amplitude), omega(omega) {
	}
	virtual ~J2control() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type jp;
	double j2_0;
	double amp, omega;
	double theta;

	virtual void operate() {
		theta = omega * this->input.getValue();
		jp[3] = amp* std::sin(theta);;//(3.14/2) + amp* std::sin(theta);// + j2_0;
//		jt[3] = 3.0 * std::sin(theta) + j4_0;
		//jt[3] = 1.5;

		this->outputValue->setData(&jp);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(J2control);
};





template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

//
	typedef boost::tuple<double, jp_type, jv_type, ja_type, jt_type, jt_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, ja_type,  jt_type, jt_type> tg_type;
	tg_type tg;

	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}







//
	double Frequency;
	std::cout << "Enter Frequency, it is advisable to set it low values ";
	std::cin >> Frequency;
	std::cout << "\n Enter Amplitude , it must be between zero to pi/2 or 1.57";
	double Amplitude;
	std::cin >> Amplitude;

//	const double JT_AMPLITUDE = 3; // N*m
//	const double FREQUENCY =1; // rad/s
//
//	if (Amplitude*Frequency > 5 && Frequency > 3)
//	{
//		Amplitude = 4;
//		Frequency = 1;
//	}


	const double JT_AMPLITUDE = Amplitude;
	const double FREQUENCY = Frequency;

	double omega1;

	if(argc==4){
		  omega1 = boost::lexical_cast<double>(argv[2]);

		        }
		else{
		  omega1 =100;

		}
		std::cout<<"Omega_P1: "<<omega1 << std::endl;


	wam.gravityCompensate();

	const double TRANSITION_DURATION = 0.5; // seconds



	//jp_type startPosition(0.0);
	jp_type startpos(0.0);
	//startpos[3] = +1.57;


	systems::Ramp time(pm.getExecutionManager(), 1.0);
	printf("Press [Enter] to turn on torque control to go to zero position");
	wam.moveTo(startpos);


	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();


	// getJointTorque from the systems
//	wam.getJointTorques()
//	system::JointTorque<jt_type>





	//==============================================
	// Joint acc calculator
	systems::FirstOrderFilter<jv_type> filter;
	wam.jvFilter.setLowPass(jv_type(omega1));
		filter.setHighPass(jp_type(omega1), jp_type(omega1));
	pm.getExecutionManager()->startManaging(filter);
	systems::Gain<jv_type, double, ja_type> changeUnits1(1.0);
	systems::connect(wam.jvOutput, filter.input);
	systems::connect(filter.output, changeUnits1.input);

	//
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
	//

	J2control<DOF> jtc(startpos, JT_AMPLITUDE, FREQUENCY);
	systems::connect(tg.output, logger.input);
	systems::connect(time.output, jtc.input);
	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(wam.jvOutput, tg.template getInput<2>());
	systems::connect(changeUnits1.output, tg.template getInput<3>());
//	systems::connect(wam.jtSum.output, tg.template getInput<4>());
	systems::connect(wam.supervisoryController.output, tg.template getInput<4>());
	systems::connect(wam.gravity.output,tg.template getInput<5>());

//	systems::connect(wam.jpController.controlOutput,tg.template getInput<3>());
//	systems::connect(wam.jtSum.output, tg.template getInput<4>());

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
/*
 * theta2sinu.cpp
 *
 *  Created on: 24-Jan-2015
 *      Author: nilxwam
 */




/*
 * theta_2_4.cpp
 *
 *  Created on: 27-Jan-2015
 *      Author: nilxwam
 */




