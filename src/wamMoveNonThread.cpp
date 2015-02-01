/* This code collect data for Hybrid Learning
 * It stores B*u along with other data. where B=inv(MassMat)
 */


#include <iostream>
#include <string>
#include <ctime>
#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()
#include <sys/time.h>
#include <boost/tuple/tuple.hpp>
#include <boost/lexical_cast.hpp>
#include <barrett/log.h>
#include <barrett/units.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include<myUtilWAMwrapper.h>

#include <massMatrixHolder.hpp>

/* Auxiliary System (Computes M!) */
#include <dp_DynamicsBaseSystem.hpp>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

using namespace barrett;
using detail::waitForEnter;

bool validate_args(int argc,char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <File Write>" << " <vel Filter1 cutoff freq.>"
        << " < Acc Filter1 cutoff freq.>" << " <no. of loops>" << " <mFact.>" << std::endl;
    return false;
  }
  return true;
}

template<size_t DOF>
  int wam_main(int argc,char** argv,ProductManager& pm,systems::Wam<DOF>& wam) {
  const double TRANSITION_DURATION = 0.1;
  double mFact;
  const int NSTATES = 4;
  const int NR = 7;
  double omega;
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  typedef typename systems::massMatrixHolder<DOF, NSTATES/2>::bu2D_type bu2D_type;
  typedef typename systems::massMatrixHolder<DOF, NSTATES/2>::bu_type bu_type;
  typedef systems::compTorqueController<NSTATES, NR, jp_type, jv_type, jt_type> compController_type;
//  typedef systems::TupleGrouper<double, jp_type, jv_type, jt_type, ja_type, bu2D_type, bu_type> tg_type;
  typedef systems::TupleGrouper<bu2D_type> tg_type;


  srand(time(NULL));
  float ThMax1, ThMax3;
  float ThMin1, ThMin3;
  float convertion = 0.0174532925;
  ThMax1 = 15 * convertion;
  ThMin1 = -15 * convertion;
  ThMax3 = 15 * convertion;
  ThMin3 = -15 * convertion;

  size_t loop = 1;
  double ts = pm.getExecutionManager()->getPeriod();
  std::cout << "TS:  " << ts << std::endl;

  char tmpFile[] = "/tmp/btXXXXXX";
  if (mkstemp(tmpFile) == -1) {
    printf("ERROR: Couldn't create temporary file!\n");
    return 1;
  }

  if (argc>2) {
    omega = boost::lexical_cast<double>(argv[2]);
  } else {
    omega = 100;
  }

  if (argc >4) {
    loop = boost::lexical_cast<size_t>(argv[4]);
  }
  if (argc >5) {
    mFact = boost::lexical_cast<double>(argv[5]);

  } else {

    mFact = 4;
  }

  jp_type desp(0.0), jp(0.0);


  // Create our system - this needs our default PID settings from the configuration file
  const libconfig::Setting& setting = pm.getConfig().lookup(pm.getWamDefaultConfigPath());

  compController_type compCon(setting["joint_position_control"]);

  systems::Ramp time(pm.getExecutionManager(), 1.0); //slope 1/ts will give increment 1 for eash time step

  wam.jvFilter.setLowPass(jv_type(omega));
  wam.gravityCompensate();

  jp_type startPos(wam.getJointPositions());


  // MassMatrix System
  DynamicsBaseSystem<DOF> my_dynamics(pm.getExecutionManager(),pm.getConfig().lookup(pm.getWamDefaultConfigPath()),
      wam.getJointVelocities(),wam.getJointPositions());
  systems::connect(wam.jpOutput, my_dynamics.jpInput);
  systems::connect(wam.jvFilter.output, my_dynamics.jvInput);
  systems::massMatrixHolder<DOF, NSTATES/2> mh(pm.getExecutionManager());
  systems::connect(my_dynamics.output, mh.MqInput);
  systems::connect(wam.jpController.controlOutput, mh.uInput);

  tg_type tg;

  //==============================================
  // Joint acc calculator
  systems::FirstOrderFilter<jv_type> filter;


  if (argc >3) {
    omega = boost::lexical_cast<double>(argv[3]);
  } else {
    omega = 100;
  }

  filter.setHighPass(jp_type(omega), jp_type(omega));
  pm.getExecutionManager()->startManaging(filter);

  systems::Gain<jv_type, double, ja_type> changeUnits1(1.0);
  systems::connect(wam.jvOutput, filter.input);
  systems::connect(filter.output, changeUnits1.input);
  systems::connect(wam.jpOutput, compCon.feedbackInput);
  systems::connect(wam.jvFilter.output, compCon.wamJVIn);
     // Register our controller as the default PID controller for Joint Position Control
//  wam.supervisoryController.registerConversion(
//      systems::makeIOConversion(compCon.referenceInput, compCon.controlOutput));

  //Position and velocity error generator



//  systems::connect(time.output, tg.template getInput<0>());
//  systems::connect(wam.jpOutput, tg.template getInput<1>());
//  systems::connect(wam.jvOutput, tg.template getInput<2>());
//  systems::connect(compCon.controlOutput, tg.template getInput<3>());
//  systems::connect(changeUnits1.output, tg.template getInput<4>());
  systems::connect(mh.BuOutput2D, tg.template getInput<0>());
//  systems::connect(mh.BuOutput, tg.template getInput<6>());
//  typedef boost::tuple<double, jp_type, jv_type, jt_type, ja_type, bu2D_type, bu_type> tuple_type;

  typedef boost::tuple<bu2D_type>tuple_type;
  const size_t PERIOD_MULTIPLIER = 1;
  systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
      new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
      PERIOD_MULTIPLIER);

  for (size_t i = 0; i < loop; i++) {
    std::cout << "joint position: " << jp << std::endl;
    jp[1] = (ThMin1 + (ThMax1 - ThMin1) * float(rand()) / RAND_MAX);
    jp[3] = (ThMin3 + (ThMax3 - ThMin3) * float(rand()) / RAND_MAX);

//                          if (str.compare("neq")==0){
//    if (i > 8) {
//      desp[1] = (ThMin1 + (ThMax1 - ThMin1) * float(rand()) / RAND_MAX);
//      desp[3] = (ThMin3 + (ThMax3 - ThMin3) * float(rand()) / RAND_MAX);
//    }
//    if (i > 3 && i < 8) {
//      if (i == 4) {
//        jp[1] = ThMax1;
//        jp[3] = ThMax3;
////        desp[1] = ThMin1;
////        desp[3] = ThMin3;
//      }
//      if (i == 5) {
//        jp[1] = ThMax1;
//        jp[3] = ThMin3;
////        desp[1] = ThMin1;
////        desp[3] = ThMax3;
//      }
//      if (i == 6) {
//        jp[1] = ThMin1;
//        jp[3] = ThMax3;
////        desp[1] = ThMax1;
////        desp[3] = ThMin3;
//      }
//      if (i == 7) {
//        jp[1] = ThMin1;
//        jp[3] = ThMin3;
////        desp[1] = ThMax1;
////        desp[3] = ThMax3;
//      }
//    }

    if (i == 0) {
      jp[1] = ThMin1;
      jp[3] = ThMin3;
    }
    if (i == 1) {
      jp[1] = ThMin1;
      jp[3] = ThMax3;
    }
    if (i == 2) {
      jp[1] = ThMax1;
      jp[3] = ThMin3;
    }
    if (i == 3) {
      jp[1] = ThMax1;
      jp[3] = ThMax3;
    }
    std::cout << "moving to: " << jp << std::endl;

    
    myUtilWAMwrapper<jp_type, compController_type, tg_type, DOF> wamRap(&compCon, &wam, &tg, &time);
    
    systems::connect(tg.output, logger.input);
    wamRap.MoveWAMto(desp, mFact, 0.5);
    systems::disconnect(logger.input);
  }
  logger.closeLog();
  wam.moveHome();
  // Wait for the user to press Shift-idle
  pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);



  printf("Logging stopped.\n");

  log::Reader<tuple_type> lr(tmpFile);
  lr.exportCSV(argv[1]);
  printf("Output written to %s.\n", argv[1]);
  std::remove(tmpFile);


  return 0;


}
