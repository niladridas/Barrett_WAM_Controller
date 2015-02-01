#define DEBUG_GMM_
#include <iostream>
#include <string>

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <GMMTrajectory.h>
#include <barrett/log.h>
//#include <massMatrixHolder.hpp>

#include <barrett/standard_main_function.h>


using namespace barrett;

class CpCircle : public systems::SingleIO<double, units::CartesianPosition::type> {
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
    CpCircle(cp_type startPos, double amplitude, double omega, const std::string& sysName = "CpCircle") :
        systems::SingleIO<double, cp_type>(sysName), cp(startPos), x_0(cp[0]), y_0(cp[1]), amp(amplitude), omega(omega) {}
    virtual ~CpCircle() { mandatoryCleanUp(); }

protected:
    cp_type cp;
    double x_0, y_0;
    double amp, omega;
    double theta;

    virtual void operate() {
        theta = omega * this->input.getValue();

        cp[0] = amp * (std::cos(theta) - 1.0) + x_0;
        cp[1] = amp * std::sin(theta) + y_0;

        this->outputValue->setData(&cp);
    }

private:
    DISALLOW_COPY_AND_ASSIGN(CpCircle);
};
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	typedef isl::GMMTrajectory::vec3D vec3D;
	typedef boost::tuple<cp_type, cv_type, cp_type, cv_type, double, jp_type, vec3D, vec3D> tuple_type;
	typedef systems::TupleGrouper<cp_type, cv_type,cp_type, cv_type, double, jp_type, vec3D, vec3D> tg_type;
	tg_type tg;
	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
	  printf("ERROR: Couldn't create temporary file!\n");
	  return 1;
	}
	double omega;
	if (argc>2) {
	  double tmp = boost::lexical_cast<double>(argv[2]);
	  if(tmp>0 && tmp<180){
	    omega = tmp;

	  }
	} else {
	  omega = 50.0;
	}

	size_t max_while_cnt;
	if (argc>3) {
	  double tmp = boost::lexical_cast<size_t>(argv[3]);
	  if(tmp>0){
	    max_while_cnt = tmp;

	  }
	} else {
	  max_while_cnt = 1000;
	}
	std::cout<< "##################################\nLow pass freq. is set to: "<< omega<<"\n"<<" Max_WHILE_CNT: "<<max_while_cnt<<"\n##############################\n";
	const double CP_AMPLITUDE = 0.1;  // meters
	const double FREQUENCY = 1.0;  // rad/s
	const double TRANSITION_DURATION = 0.5;  // seconds
	const double JP_AMPLITUDE = 0.4;  // radians
	wam.gravityCompensate();

	jp_type startPos(0.0), posZero(0.0);
//	startPos[1] = -M_PI_2;
//	startPos[3] = M_PI_2 + JP_AMPLITUDE;

	startPos[0]=1.05;
	startPos[1]=0.32;
	startPos[3] = 3;

	cp_type xi;
	xi[0]=0.697;
	xi[1]= - 0.012;
	xi[2]= - 0.19;

	systems::Ramp time(pm.getExecutionManager(), 1.0);



	printf("Press [Enter] to follow traj.");
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type > logger(pm.getExecutionManager(),
	    new log::RealTimeWriter<tuple_type > (tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
	    PERIOD_MULTIPLIER);

	isl::GMMTrajectory gt(pm.getExecutionManager(), 0.0,0.01);
	gt.setLowpass(omega);
	CpCircle cpc(wam.getToolPosition(), CP_AMPLITUDE, FREQUENCY);
	systems::connect(wam.toolPosition.output,gt.cp_input);
//	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(gt.cp_output, tg.template getInput<0>());
	systems::connect(gt.cv_output, tg.template getInput<1>());
	systems::connect(wam.toolPosition.output, tg.template getInput<2>());
	systems::connect(gt.gmm_out, tg.template getInput<3>());
	systems::connect(gt.diff_norm_out, tg.template getInput<4>());
	systems::connect(wam.jpOutput, tg.template getInput<5>());
	systems::connect(gt.f_out, tg.template getInput<6>());
	systems::connect(gt.u_out, tg.template getInput<7>());

	wam.moveTo(posZero);
	wam.moveTo(startPos);


	systems::connect(tg.output, logger.input);
//	time.reset();
//	systems::connect(time.output, cpc.input);
//	wam.trackReferenceSignal(cpc.output);
//	time.smoothStart(TRANSITION_DURATION);
//	printf("Press [Enter] to stop.");
//	Sam::waitForEnter();
//	time.smoothStop(TRANSITION_DURATION);
	time.start();
	pm.getExecutionManager()->startManaging(gt);


	btsleep(.01);
	Sam::waitForEnter();
	wam.trackReferenceSignal(gt.cp_output);
	gt.start(wam.getToolPosition());
	size_t cnt=0;
	while((xi-wam.getToolPosition()).norm()>0.1){
	    btsleep(0.00001);
	    cnt++;
	    if(cnt>max_while_cnt){
	      break;
	    }
	}
	gt.stop();
	logger.closeLog();
	wam.moveTo(posZero);
	wam.moveHome();
	wam.idle();




//
//	printf("Press [Enter] to stop.");
//	Sam::waitForEnter();
//	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);
	return 0;
}
