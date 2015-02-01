
#define INTE_LYAP


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
#include <myController.h>
//#include<myUtilWAMwrapper.h>
#include <samlibs.h>
//#include <dp_DynamicsBaseSystem.hpp>
#include <massMatrixHolder.hpp>
#include <BmatrixGenerator.hpp>
#include <costComputeCritic.hpp>
/* Auxiliary System (Computes M!) */


#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

using namespace barrett;
//using systems::detail::waitForEnter;

bool validate_args(int argc,char** argv) {
  if (argc <2) {
    std::cout << "Usage: " << argv[0] << " <File Write>" << " <vel Filter1 cutoff freq.>"
        << " < Acc Filter1 cutoff freq.>" << " <no. of loops>" <<" <Joint velocity in rad>"<<" <Joint ACC in rad>" << " <Kp_omega and Kd_omega.>" << std::endl;
    return false;
  }
  return true;
}

template<size_t DOF>
  int wam_main(int argc,char** argv,ProductManager& pm,systems::Wam<DOF>& wam) {


  const int NJOINTS = 2;
  const int NR = 9;
  double omega, omega1, j_vel, j_acc;
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  typedef systems::myController<NJOINTS, NR, jp_type, jv_type, jt_type> myController_type;
  typedef typename myController_type::criticTorque_type cTorque_type;
  typedef typename myController_type::vec_type extra_data_type;
  typedef typename myController_type::unitless_type_jp err_type;
  typedef typename Sam::massMatrixHolder<DOF, NJOINTS>::mMat7D_type mMat7D_type;
  typedef typename Sam::massMatrixHolder<DOF, NJOINTS>::mMat2D_type mMat2D_type;
  typedef typename Sam::BmatrixGenerator<DOF,NJOINTS>::BmatAsVec_type bvec_type;
//  typedef systems::TupleGrouper<double, jp_type, jv_type, err_type, err_type, jt_type , ja_type> tg_type;
  typedef systems::TupleGrouper<double, jp_type, jv_type, jt_type, cTorque_type,double,double> tg_type;



  srand(time(NULL));
  float ThMax1, ThMax3;
  float ThMin1, ThMin3;
  float convertion = 0.0174532925;
  ThMax1 = 45* convertion;
  ThMin1 = -45 * convertion;
  ThMax3 = 45 * convertion;
  ThMin3 = -30 * convertion;

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
  if (argc >3) {
      omega1 = boost::lexical_cast<double>(argv[3]);
    } else {
      omega1 = 100;
    }

  if (argc >4) {
    loop = boost::lexical_cast<size_t>(argv[4]);
  }

  wam.jvFilter.setLowPass(jv_type(omega));
  wam.gravityCompensate();

  jp_type desp(0.0), jp(0.0);

  // Create our system - this needs our default PID settings from the configuration file
  const libconfig::Setting& setting = pm.getConfig().lookup(pm.getWamDefaultConfigPath());

  myController_type myCon(setting["joint_position_control"]);
  std::cout<<"Rmat:\n"<<myCon.getRinv()<<"\n";
  if (argc ==9) {
    myCon.setKpKd_comp(boost::lexical_cast<double>(argv[7]),boost::lexical_cast<double>(argv[8]));

  }
  else{
    myCon.setKpKd_comp(57,10);
  }

  if (argc ==6) {
    j_vel=boost::lexical_cast<double>(argv[5]);
  }
  else{
    j_vel=0.5;
  }
  if (argc ==7) {
      j_acc=boost::lexical_cast<double>(argv[6]);
    }
    else{
      j_acc=0.5;
    }


  systems::Ramp time(pm.getExecutionManager(), 1.0/ts); //slope 1/ts will give increment 1 for eash time step


  jp_type startPos(wam.getJointPositions());

   //////MassMatrix System
  Sam::massMatrixHolder<DOF, NJOINTS> mm(pm.getExecutionManager(),pm.getConfig().lookup(pm.getWamDefaultConfigPath()),wam.getJointPositions(),
      wam.getJointVelocities());
  systems::connect(wam.jpOutput, mm.jpInput);
  systems::connect(wam.jvFilter.output, mm.jvInput);
  systems::connect(mm.mInv2JOutput, myCon.mMat2Dinp);

  Sam::BmatrixGenerator<DOF, NJOINTS> bg(pm.getExecutionManager());
  systems::connect(myCon.controlOutput, bg.controlInp);
  systems::connect(mm.mMat7DOutput, bg.inputMassMat);

  Sam::costComputeCritic<DOF,NJOINTS,NR> costComputer(pm.getExecutionManager(),"FuzzParams/P1.txt","FuzzParams/P75.txt");
  systems::connect(wam.jpOutput,costComputer.jPosInp);
  systems::connect(wam.jvFilter.output,costComputer.jVelInp);

  tg_type tg;

  //==============================================
  // Joint acc calculator
  systems::FirstOrderFilter<jv_type> filter;
  systems::FirstOrderFilter<cTorque_type> jtfilter;
  jtfilter.setLowPass(cTorque_type(omega1));
  systems::connect(myCon.criticconOp, jtfilter.input);
  pm.getExecutionManager()->startManaging(jtfilter);
  filter.setHighPass(jp_type(omega1), jp_type(omega1));
  pm.getExecutionManager()->startManaging(filter);

  systems::Gain<jv_type, double, ja_type> changeUnits1(1.0);
//  pm.getExecutionManager()->startManaging(changeUnits1);
  systems::connect(wam.jvFilter.output, filter.input);
  systems::connect(filter.output, changeUnits1.input);
  systems::connect(wam.jpOutput, myCon.feedbackInput);
  systems::connect(wam.jvFilter.output, myCon.wamJVIn);
//  systems::connect(mg.BmatOutput2J,criticCon.bMat2Dinp);
     // Register our controller as the default PID controller for Joint Position Control
//  wam.supervisoryController.registerConversion(
//      systems::makeIOConversion(myCon.referenceInput, myCon.controlOutput));



  systems::connect(time.output, tg.template getInput<0>());
  systems::connect(wam.jpOutput, tg.template getInput<1>());
  systems::connect(wam.jvFilter.output, tg.template getInput<2>());

//
  systems::connect(myCon.controlOutput, tg.template getInput<3>());
  systems::connect(jtfilter.output, tg.template getInput<4>());
  systems::connect(costComputer.cost1Output,tg.template getInput<5>());
  systems::connect(costComputer.cost2Output,tg.template getInput<6>());
//  systems::connect(myCon.dataVecOutput,tg.template getInput<3>());
//  systems::connect(bg.BmatAsVecOutput, tg.template getInput<5>());
//  systems::connect(myCon.criticconOp, tg.template getInput<1>());
//  systems::connect(myCon.criticconOp, tg.template getInput<4>());

  typedef boost::tuple<double, jp_type, jv_type, jt_type, cTorque_type,double, double> tuple_type;

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

    wam.moveTo(jp);
    wam.supervisoryController.registerConversion(
          systems::makeIOConversion(myCon.referenceInput, myCon.controlOutput));
    systems::connect(tg.output, logger.input);
    btsleep(1000000);
    time.start();
    wam.moveTo(desp,j_vel, j_acc);
    time.stop();
    systems::disconnect(logger.input);

    time.setOutput(0.0);
    myCon.resetRcount();
    wam.supervisoryController.registerConversion(
             systems::makeIOConversion(wam.jpController.referenceInput, wam.jpController.controlOutput));
    //waitForEnter();
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
