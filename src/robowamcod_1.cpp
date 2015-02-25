
#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>
#include <samlibs.h>
#include <robowamcod_1.hpp>


using namespace barrett;
using detail::waitForEnter;

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) 
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	DMPCONTROL dmprobo<DOF>;
	
	//-----------------------------------------------------------
	float runtime = 5.0; // sec
	float samp_time = 0.0025; // 2.5 millisec
	float a[3] = {25,25,25}; // PD values
	float b[3] = {6.25,6.25,6.25};  // PD values	 
	float y0[3] = {0,0,0.2}; // Initial state [x0,y0,z0]
	float goal[3] = {0,0,0.5}; // Final state [xf,yf,zf]
	//-----------------------------------------------------------

	dmprobo.dmp.InitDmpSys(3, 10, a, b, runtime, 0.05);
	dmprobo.dmp.SetDMPConditions(y0, goal); // Initial conditions
	dmprobo.dmp.CheckDMPGaolOffset(); 

	typedef boost::tuple<double, cp_type, cv_type> tuple_type;
	typedef systems::TupleGrouper<double, cp_type, cv_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	
	if (mkstemp(tmpFile) == -1) 
	{
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	
	cp_type startpos;
	startpos[0] = 0;
	startpos[1] = 0;
	startpos[2] = 0.2;

	wam.gravityCompensate();
	printf("Press [Enter] to turn on torque control to go to zero position");
	waitForEnter();

	wam.moveTo(startpos);
	printf("Press [Enter] to turn on torque control to joint 2.");
	waitForEnter();
	
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(), new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()), PERIOD_MULTIPLIER);

	systems::connect(tg.output, logger.input);
	systems::connect(wam.cpOutput,dmprobo.CurrentStatePos)
	systems::connect(wam.cvOutput,dmprobo.CurrentStateVel)
	wam.trackReferenceSignal(dmprobo.NextState);

	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(joint_ref.referencejpTrack, tg.template getInput<1>());
	systems::connect(joint_ref.referencejvTrack, tg.template getInput<2>());
	

	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
//	printf("It all ended");
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);
			log.close();
	return 0;
}

