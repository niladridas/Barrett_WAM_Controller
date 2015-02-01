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
class J2control: public systems::SingleIO<double,
		typename units::JointPositions<DOF>::type> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	J2control(jp_type startPosition, double amplitude, double omega,
			double offset, const std::string& sysName = "JpSinusoid") :
			systems::SingleIO<double, jp_type>(sysName), jp(startPosition), j2_0(
					jp[1]), amp(amplitude), omega(omega), offset(offset) {
	}
	virtual ~J2control() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type jp;
	double j2_0;
	double amp, omega, offset;
	double theta;

	virtual void operate() {
		theta = omega * this->input.getValue();
		jp[1] = offset + amp * std::sin(theta);

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
	typedef systems::TupleGrouper<double, jp_type, jv_type, ja_type, jt_type,
			jt_type> tg_type;
	tg_type tg;

	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	double Frequency;
	double Amplitude;

	double omega1;
	double offset;

	if (argc == 2) {
		Frequency = 0.1;
		Amplitude = 0.525;
		offset = 0;
	}
	if (argc == 3) {
		Amplitude = 0.525;
		offset = 0;
	}
	if (argc == 4) {
		offset = 0;
	}

	else {
		Frequency = boost::lexical_cast<double>(argv[2]);
		Amplitude = boost::lexical_cast<double>(argv[3]);
		offset = boost::lexical_cast<double>(argv[4]);
	}





	const double JT_AMPLITUDE = Amplitude;
	const double FREQUENCY = Frequency;
	const double OFFSET = offset;

	omega1 = 120;

	wam.gravityCompensate();

	const double TRANSITION_DURATION = 0.5; // seconds

	jp_type startpos(0.0);
	startpos[3] = +3.14;
	startpos[1] = offset;

	systems::Ramp time(pm.getExecutionManager(), 1.0);
	printf("Press [Enter] to turn on torque control to go to zero position");
	wam.moveTo(startpos);

	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();

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

	J2control<DOF> jtc(startpos, JT_AMPLITUDE, FREQUENCY, OFFSET);
	systems::connect(tg.output, logger.input);
	systems::connect(time.output, jtc.input);
	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(wam.jvOutput, tg.template getInput<2>());
	systems::connect(changeUnits1.output, tg.template getInput<3>());
	systems::connect(wam.supervisoryController.output,
			tg.template getInput<4>());
	systems::connect(wam.gravity.output, tg.template getInput<5>());

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

